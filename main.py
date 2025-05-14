# main_automation_app.py

import sys
import subprocess
import threading # subprocess 관리를 위해 threading도 일부 사용
from argparse import ArgumentParser
from typing import Optional, List, Tuple, Dict
from xml.etree import ElementTree

from PySide6.QtCore import Qt, QThread, Signal, Slot, QSize, QEvent, QRect, QPoint
from PySide6.QtGui import QImage, QKeyEvent, QMouseEvent, QPixmap, QPainter, QColor, QBrush, QPen, QFont, QPalette
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QMessageBox, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QTextEdit, QComboBox, QCheckBox, QGroupBox, QFormLayout, QToolTip
)

from adbutils import adb, AdbError
import scrcpy
from PIL import Image # For MobileCLIP if it needs PIL Images
import io

# --- FastVLM 가상 Wrapper ---
class FastVlmModelWrapper:
    def __init__(self, model_path="checkpoints/fastvlm_0.5b_stage3"): 
        self.model_path = model_path
        self.is_ready = True 
        print(f"Initializing FastVLM (conceptual) with model path: {self.model_path}...")

    def _simulate_vlm_action_object_extraction(self, command_text: str) -> Dict:
        words = command_text.split()
        action = None
        target_object_description = command_text # Default
        text_to_type = None

        known_actions = {
            "click": "Click", "tap": "Click", "press": "Click", "select": "Click", "open": "Click",
            "type": "Type", "enter": "Type", "input": "Type", "write": "Type", "put": "Type",
            "scroll": "Scroll", "swipe": "Swipe", "move": "Move",
            "find": "Find", "search": "Find", "look for": "Find"
        }
        # Multi-word actions like "look for" or "type in" should be handled carefully.
        # For simplicity, we check for leading keywords.
        
        # Try to parse "type 'text' in/into 'target'" or "type 'text'"
        if words and words[0].lower() in [k for k,v in known_actions.items() if v == "Type"]:
            action = known_actions[words[0].lower()]
            # Search for "in" or "into" to separate text and target
            text_parts = []
            target_parts = []
            found_in_into = False
            for i in range(1, len(words)):
                if words[i].lower() in ["in", "into"] and i + 1 < len(words):
                    # Assume words between action and "in/into" are text_to_type
                    text_parts = words[1:i]
                    target_parts = words[i+1:]
                    found_in_into = True
                    break
            
            if found_in_into and text_parts:
                text_to_type = " ".join(text_parts)
                target_object_description = " ".join(target_parts)
            elif len(words) > 1: # No "in/into", so treat rest as text, and also as target for now
                text_to_type = " ".join(words[1:])
                target_object_description = text_to_type # VLM needs to be smart to find an input field based on this
            else: # Only action word, e.g., "type" - not enough info
                target_object_description = ""
                text_to_type = None # Or prompt user for text?
        else: # Other actions or no specific action keyword
            # Basic action extraction as before
            for i in range(min(len(words), 3), 0, -1):
                potential_action_phrase = " ".join(words[:i]).lower()
                if potential_action_phrase in known_actions:
                    action = known_actions[potential_action_phrase]
                    target_object_description = " ".join(words[i:])
                    break
            if not action and words: # Default to first word if no known action
                action = words[0].capitalize()
                target_object_description = " ".join(words[1:])
            elif not words:
                action = "Unknown"
                target_object_description = ""

        print(f"[FastVLM Sim] Cmd: '{command_text}' -> Act='{action}', Target='{target_object_description}', Text='{text_to_type}'")
        return {"action": action, 
                "target_object_description": target_object_description.strip(),
                "text_to_type": text_to_type.strip() if text_to_type else None}

    def find_element_by_text(self, screen_pil_image: Image.Image, ui_dump_elements: List[Dict], command_text: str) -> Optional[Dict]:
        if not self.is_ready or not ui_dump_elements:
            print("[FastVLM Wrapper] Not ready or no UI elements.")
            return None

        vlm_interpretation = self._simulate_vlm_action_object_extraction(command_text)
        action_from_vlm = vlm_interpretation.get("action")
        target_desc_from_vlm = vlm_interpretation.get("target_object_description")
        text_to_type_from_vlm = vlm_interpretation.get("text_to_type")

        if not target_desc_from_vlm and action_from_vlm != "Type": # For Type, target_desc might be empty if text_to_type is the target
            print(f"[FastVLM Wrapper] Could not determine target object from command: '{command_text}'")
            return None
        
        # If action is Type and target_desc is empty, but text_to_type exists, use text_to_type to find the element
        search_description = target_desc_from_vlm
        if action_from_vlm == "Type" and not target_desc_from_vlm and text_to_type_from_vlm:
            search_description = text_to_type_from_vlm
            print(f"[FastVLM Wrapper] Type action: Using text_to_type ('{text_to_type_from_vlm}') to find target element.")
        elif not search_description: # Still no search description
             print(f"[FastVLM Wrapper] No valid object description to search for command: '{command_text}'")
             return None

        print(f"[FastVLM Wrapper] Searching for element matching: '{search_description}'")

        best_element = None
        highest_similarity = -1.0
        search_keywords = set(kw for kw in search_description.lower().split() if len(kw) > 1)

        for element in ui_dump_elements:
            is_clickable = element.get("clickable", "false") == "true"
            is_focusable = element.get("focusable", "false") == "true"
            # For type action, target should ideally be an EditText or similar (focusable, enabled)
            if action_from_vlm == "Type":
                if not (is_focusable and element.get("enabled", "false") == "true") :
                    continue # Must be focusable and enabled for typing
            elif not (is_clickable or is_focusable): # For other actions
                continue

            bounds = element.get("bounds_rect")
            if not bounds or bounds[2] <= 0 or bounds[3] <= 0: 
                continue
            
            element_text_content = (element.get("text", "") + " " + element.get("content-desc", "")).lower().strip()
            element_resource_id = element.get("resource-id", "").lower()
            element_class = element.get("class", "").lower()
            
            current_similarity = 0.0
            
            words_in_element_text = set(w for w in element_text_content.split() if len(w) > 1)
            common_keywords_text = search_keywords.intersection(words_in_element_text)
            current_similarity += float(len(common_keywords_text)) * 1.0

            for search_kw in search_keywords:
                if search_kw in element_resource_id:
                    current_similarity += 1.5 
            
            if search_description.lower() == element_text_content:
                current_similarity += 3.0 
            elif search_description.lower() in element_text_content:
                current_similarity += 1.0 
            
            if search_description.lower() in element_resource_id:
                 current_similarity += 2.0 
            
            # Bonus if the class name indicates an input field for Type actions
            if action_from_vlm == "Type" and ("edittext" in element_class or "textfield" in element_class or "input" in element_class):
                current_similarity += 1.0

            if current_similarity > 0:
                 print(f"[FastVLM Sim] Element: {element.get('resource-id', element_text_content[:30])}, Score: {current_similarity:.2f} (for obj: '{search_description}')")

            if current_similarity > highest_similarity:
                highest_similarity = current_similarity
                best_element = element
        
        if best_element and highest_similarity >= 0.5: # Adjusted threshold slightly, might need more tuning
            best_element['vlm_action'] = action_from_vlm 
            best_element['vlm_text_to_type'] = text_to_type_from_vlm
            print(f"[FastVLM Sim] Best match: {best_element.get('resource-id', best_element.get('text', 'Unknown'))} with score {highest_similarity:.2f}. Action: {action_from_vlm}, Text: {text_to_type_from_vlm}")
            return best_element
        else:
            print(f"[FastVLM Sim] No suitable element found for VLM object '{search_description}'. Max score: {highest_similarity:.2f}")
            return None

# --- ADB 관련 유틸리티 ---
def get_ui_dump(device_serial: Optional[str]) -> Optional[str]:
    try:
        device = adb.device(serial=device_serial)
        # uiautomator dump는 /sdcard/window_dump.xml 에 저장됨. 이를 읽어와야 함.
        # 가끔 권한 문제가 있을 수 있으므로 /data/local/tmp 사용 시도
        temp_path = "/data/local/tmp/uidump.xml"
        device.shell(f"uiautomator dump {temp_path}")
        xml_content = device.shell(f"cat {temp_path}")
        device.shell(f"rm {temp_path}") # 임시 파일 삭제
        return xml_content
    except AdbError as e:
        print(f"ADB Error getting UI dump: {e}")
        return None
    except Exception as e:
        print(f"Error getting UI dump: {e}")
        return None


def parse_ui_dump(xml_string: str) -> List[Dict]:
    elements = []
    if not xml_string:
        return elements
    try:
        root = ElementTree.fromstring(xml_string)
        for node in root.iter():
            attrs = node.attrib
            # bounds 파싱: "[x1,y1][x2,y2]" -> (x, y, width, height)
            bounds_str = attrs.get("bounds")
            if bounds_str:
                try:
                    parts = bounds_str.replace("][", ",").replace("[", "").replace("]", "").split(',')
                    x1, y1, x2, y2 = map(int, parts)
                    attrs["bounds_rect"] = (x1, y1, x2 - x1, y2 - y1)
                except ValueError:
                    attrs["bounds_rect"] = (0,0,0,0) # 파싱 실패 시
            elements.append(attrs)
    except ElementTree.ParseError as e:
        print(f"Error parsing UI dump XML: {e}")
    return elements


# --- Screen Overlay Widget ---
class ScreenOverlayWidget(QWidget):
    # 클래스 레벨에 무시할 레이아웃 타입의 (부분) 이름 정의
    LAYOUT_CLASSES_TO_IGNORE = [
        "ViewGroup", # android.view.ViewGroup 및 그 서브클래스들 포함 가능성
        "RelativeLayout",
        "LinearLayout",
        "FrameLayout",
        "GridLayout", # GridView 대신 GridLayout이 더 일반적일 수 있음, GridView도 포함
        "GridView",
        "ScrollView",
        "HorizontalScrollView",
        "androidx.viewpager.widget.ViewPager",
        "androidx.recyclerview.widget.RecyclerView",
        "android.webkit.WebView",
        "AppWidgetHostView",
        "View",
        "RecyclerView"
        
        # 필요에 따라 더 추가
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui_elements: List[Dict] = []
        self.filtered_ui_elements: List[Dict] = [] # 필터링된 요소 저장
        self.pixmap_offset_x = 0.0
        self.pixmap_offset_y = 0.0
        self.pixmap_width = 0
        self.pixmap_height = 0

        self.current_hover_element: Optional[Dict] = None
        self.setMouseTracking(True) 
        self.setAttribute(Qt.WA_TransparentForMouseEvents, False) 
        self.setAttribute(Qt.WA_StyledBackground, True) 
        self.setStyleSheet("background-color: transparent;")

    def set_ui_elements(self, elements: List[Dict]):
        self.ui_elements = elements # 원본은 유지
        self.filtered_ui_elements.clear()
        
        # LAYOUT_CLASSES_TO_IGNORE에 있는 항목들을 소문자로 미리 변환
        lower_ignored_classes = [ignore_name.lower() for ignore_name in self.LAYOUT_CLASSES_TO_IGNORE]

        for el in elements:
            full_class_name = el.get("class", "")
            if not full_class_name:
                # 클래스 이름이 없는 요소는 일단 포함 (또는 특정 정책에 따라 제외)
                self.filtered_ui_elements.append(el)
                continue

            # 간단한 클래스 이름 추출 (예: "android.widget.LinearLayout" -> "LinearLayout")
            simple_class_name = full_class_name.split('.')[-1]
            simple_class_name_lower = simple_class_name.lower()
            
            should_ignore = False
            if simple_class_name_lower in lower_ignored_classes:
                should_ignore = True
            
            if not should_ignore:
                self.filtered_ui_elements.append(el)
        
        self.log_dev_message(f"Overlay: Original elements: {len(self.ui_elements)}, Filtered elements: {len(self.filtered_ui_elements)}")
        self.current_hover_element = None 
        QToolTip.hideText() 
        self.update()

    def log_dev_message(self, message: str):
        # 이 위젯에서 직접 MainWindow의 로그 기능을 호출할 수 없으므로,
        # 간단히 print하거나, 시그널을 통해 MainWindow로 메시지를 전달해야 함.
        # 여기서는 간단히 print로 대체.
        print(f"[ScreenOverlayWidget] {message}")

    def update_geometry_info(self, scale: float, offset_x: float, offset_y: float, pix_w: int, pix_h: int):
        self.pixmap_offset_x = offset_x
        self.pixmap_offset_y = offset_y
        self.pixmap_width = pix_w
        self.pixmap_height = pix_h
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        # 필터링된 요소 사용
        if not self.filtered_ui_elements:
            return

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        pen_color = QColor(255, 0, 0, 180) 
        hover_pen_color = QColor(0, 255, 0, 200) 

        font = QFont()
        font.setPointSize(8) 
        painter.setFont(font)

        for element in self.filtered_ui_elements: # 필터링된 요소 사용
            scaled_bounds = element.get("scaled_bounds_rect_for_pixmap")
            if not scaled_bounds:
                continue

            px, py, pw, ph = scaled_bounds 
            if pw <= 0 or ph <= 0: continue

            draw_x = px + self.pixmap_offset_x
            draw_y = py + self.pixmap_offset_y
            
            current_pen = pen_color
            if self.current_hover_element and self.current_hover_element == element:
                current_pen = hover_pen_color
            
            painter.setPen(QPen(current_pen, 1)) 
            painter.drawRect(int(draw_x), int(draw_y), int(max(1, pw)), int(max(1, ph)))

    def mouseMoveEvent(self, event: QMouseEvent):
        # 필터링된 요소 사용
        if not self.filtered_ui_elements:
            super().mouseMoveEvent(event)
            return

        mouse_pos_in_overlay = event.position() 
        mouse_x_on_pixmap = mouse_pos_in_overlay.x() - self.pixmap_offset_x
        mouse_y_on_pixmap = mouse_pos_in_overlay.y() - self.pixmap_offset_y
        
        found_element = None

        # 마우스가 QPixmap 영역 내에 있는지 확인
        if (0 <= mouse_x_on_pixmap < self.pixmap_width and 
            0 <= mouse_y_on_pixmap < self.pixmap_height):
            
            for element in reversed(self.filtered_ui_elements): # 필터링된 요소 사용
                scaled_bounds = element.get("scaled_bounds_rect_for_pixmap")
                if not scaled_bounds:
                    continue
                
                px, py, pw, ph = scaled_bounds 
                if pw <= 0 or ph <= 0: continue

                element_qrect_on_pixmap = QRect(int(px), int(py), int(max(1, pw)), int(max(1, ph)))

                if element_qrect_on_pixmap.contains(QPoint(int(mouse_x_on_pixmap), int(mouse_y_on_pixmap))):
                    found_element = element
                    break
        
        if found_element:
            if found_element != self.current_hover_element:
                self.current_hover_element = found_element
                self.update() 
                
                tooltip_text = []
                tooltip_text.append(f"<b>R_ID:</b> {found_element.get('resource-id', 'N/A')}")
                tooltip_text.append(f"<b>Text:</b> {found_element.get('text', '')[:50]}")
                tooltip_text.append(f"<b>Desc:</b> {found_element.get('content-desc', '')[:50]}")
                tooltip_text.append(f"<b>Class:</b> {found_element.get('class', '').split('.')[-1]}")
                scaled_b = found_element.get("scaled_bounds_rect_for_pixmap", "N/A")
                tooltip_text.append(f"<b>Bounds (S):</b> {scaled_b}")
                tooltip_text.append(f"<b>Clickable:</b> {found_element.get('clickable', 'false')}")
                tooltip_text.append(f"<b>Focusable:</b> {found_element.get('focusable', 'false')}")

                QToolTip.showText(event.globalPosition().toPoint(), "<br>".join(tooltip_text), self)
        else:
            if self.current_hover_element is not None:
                QToolTip.hideText()
                self.current_hover_element = None
                self.update() 

        super().mouseMoveEvent(event)

    def leaveEvent(self, event):
        QToolTip.hideText()
        self.current_hover_element = None
        self.update()
        super().leaveEvent(event)


# --- 스레드 클래스 ---
class AICommandWorker(QThread):
    status_update = Signal(str)
    log_message = Signal(str)
    action_result = Signal(str, int, int) 
    action_failed = Signal(str) 

    def __init__(self, clip_model: FastVlmModelWrapper, device_serial: Optional[str], screen_image_bytes: bytes, command: str):
        super().__init__()
        self.clip_model = clip_model
        self.device_serial = device_serial
        self.screen_image_bytes = screen_image_bytes 
        self.command = command
        self.adb_device = None
        try:
            if self.device_serial:
                self.adb_device = adb.device(serial=self.device_serial)
                self.log_message.emit(f"AIWorker: ADB device instance created: {self.adb_device.serial}")
            else:
                self.log_message.emit("AIWorker: No device serial provided for ADB connection.")
        except Exception as e:
            self.log_message.emit(f"AIWorker: Failed to initialize ADB device for {self.device_serial}: {e}")

    def run(self):
        if not self.adb_device:
            self.log_message.emit(f"AIWorker: ADB device not available (serial: {self.device_serial}). Cannot perform action.")
            self.action_failed.emit(f"ADB device {self.device_serial} not available.")
            return

        self.status_update.emit(f"Processing command: {self.command}")
        self.log_message.emit(f"AIWorker: Getting UI dump for {self.device_serial}...")

        ui_dump_xml = get_ui_dump(self.device_serial)
        if not ui_dump_xml:
            self.log_message.emit("AIWorker: Failed to get UI dump.")
            self.action_failed.emit("Failed to get UI dump.")
            return

        ui_elements = parse_ui_dump(ui_dump_xml)
        if not ui_elements:
            self.log_message.emit("AIWorker: Failed to parse UI dump or no elements found.")
            self.action_failed.emit("Failed to parse UI dump.")
            return
        
        self.log_message.emit(f"AIWorker: UI dump parsed, {len(ui_elements)} elements found.")

        try:
            screen_pil = Image.open(io.BytesIO(self.screen_image_bytes)) 
            self.log_message.emit("AIWorker: Screen image loaded for FastVLM.")
        except Exception as e:
            self.log_message.emit(f"AIWorker: Error converting screen frame to PIL Image: {e}")
            self.action_failed.emit("Error processing screen image.")
            return

        target_element = self.clip_model.find_element_by_text(screen_pil, ui_elements, self.command)

        if target_element:
            bounds = target_element.get("bounds_rect") # 원본 해상도 기준 bounds
            action_to_perform = target_element.get("vlm_action", "Click") 
            text_to_type = target_element.get("vlm_text_to_type")
            element_desc = target_element.get('text', target_element.get('content-desc', target_element.get('resource-id', 'Unknown element')))

            if not bounds or bounds[2] <= 0 or bounds[3] <= 0:
                self.log_message.emit(f"AIWorker: Found element '{element_desc}' has invalid bounds: {bounds}")
                self.action_failed.emit(f"Element '{element_desc}' has invalid bounds.")
                return

            center_x = bounds[0] + bounds[2] // 2
            center_y = bounds[1] + bounds[3] // 2
            action_message = ""

            try:
                self.log_message.emit(f"AIWorker: Attempting action '{action_to_perform}' on '{element_desc}' at ({center_x},{center_y})")
                if action_to_perform == "Click":
                    self.adb_device.click(center_x, center_y)
                    action_message = f"Clicked '{element_desc}'"
                    self.log_message.emit(f"AIWorker: ADB click performed at ({center_x},{center_y}).")
                elif action_to_perform == "Type":
                    if text_to_type is not None:
                        # 1. Click to focus the element
                        self.adb_device.click(center_x, center_y)
                        self.log_message.emit(f"AIWorker: ADB click (for focus) at ({center_x},{center_y}). Waiting briefly...")
                        QThread.msleep(300) # 짧은 지연시간으로 포커스 및 키보드 활성화 시간 확보
                        
                        # 2. Input text (주의: 특수문자 이스케이프 필요할 수 있음)
                        # adbutils의 shell 메서드는 문자열을 알아서 처리하려고 시도함.
                        escaped_text = text_to_type.replace("\"", "\\\"").replace("'", "'''''") # 기본적 쉘 이스케이프 시도
                        self.adb_device.shell(f'input text "{escaped_text}"')
                        action_message = f"Typed '{text_to_type}' into '{element_desc}'"
                        self.log_message.emit(f"AIWorker: ADB input text '{text_to_type}' (escaped: '{escaped_text}') into focused element.")
                    else:
                        action_message = f"Type action for '{element_desc}' but no text provided."
                        self.log_message.emit(f"AIWorker: {action_message}")
                        self.action_failed.emit(action_message)
                        return # 실패 처리
                else:
                    action_message = f"Action '{action_to_perform}' on '{element_desc}' is not implemented yet."
                    self.log_message.emit(f"AIWorker: {action_message}")
                    self.action_failed.emit(action_message)
                    return # 실패 처리
                
                self.action_result.emit(action_message, center_x, center_y) 

            except AdbError as e:
                err_msg = f"AIWorker: ADB Error during '{action_to_perform}' on '{element_desc}': {e}"
                self.log_message.emit(err_msg)
                self.action_failed.emit(f"ADB action failed: {e}")
            except Exception as e:
                err_msg = f"AIWorker: Unexpected error during '{action_to_perform}' on '{element_desc}': {e}"
                self.log_message.emit(err_msg)
                self.action_failed.emit(f"Action execution failed: {e}")

        else:
            self.log_message.emit(f"AIWorker: Element for '{self.command}' not found by FastVLM.")
            self.action_failed.emit(f"Could not find element for: {self.command}")

        self.status_update.emit("Command processing finished.")


class LogcatWorker(QThread):
    new_log_line = Signal(str)
    log_message = Signal(str)

    def __init__(self, device_serial: Optional[str], tag_filter: str = "", msg_filter: str = ""):
        super().__init__()
        self.device_serial = device_serial
        self.tag_filter = tag_filter.strip()
        self.msg_filter = msg_filter.strip().lower()
        self._is_running = False
        self.process = None
        self.adb_device = None
        try:
            if self.device_serial:
                self.adb_device = adb.device(serial=self.device_serial)
        except Exception as e:
            self.log_message.emit(f"LogcatWorker: Failed to connect to ADB device {self.device_serial}: {e}")


    def run(self):
        if not self.adb_device:
            self.log_message.emit(f"LogcatWorker: ADB device {self.device_serial} not available.")
            return

        self._is_running = True
        cmd = ["adb"]
        if self.device_serial:
            cmd.extend(["-s", self.device_serial])
        cmd.extend(["logcat", "-v", "brief"]) # -v tag는 태그만 보여줌. brief가 일반적

        # 태그 필터링 (adb logcat 자체 기능 활용)
        # 예: "MyApp:D *:S" -> MyApp 태그는 Debug 레벨 이상, 나머지는 Silent
        # 여기서는 간단히 모든 로그를 받고 Python에서 필터링 (더 많은 제어 가능)
        # 또는, 태그 필터가 있으면 `adb logcat YourTag:V *:S` 형태로 사용 가능
        # 여기서는 태그가 정확한 이름일 때만 필터링한다고 가정.
        # 더 복잡한 adb logcat 필터 구문은 직접 구성해야 함.
        # if self.tag_filter:
        #    cmd.append(f"{self.tag_filter}:V *:S") # 예시, 실제 필터는 더 정교해야 함

        self.log_message.emit(f"LogcatWorker: Starting logcat with command: {' '.join(cmd)}")
        try:
            # self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1, universal_newlines=True)
            # adbutils의 adb.device(...).shell("logcat ...", stream=True) 사용 가능
            stream = self.adb_device.shell("logcat -v brief", stream=True)
            self.log_message.emit("LogcatWorker: Stream opened.")

            for line in stream:
                if not self._is_running:
                    self.log_message.emit("LogcatWorker: Stopping...")
                    break
                
                line_lower = line.lower()
                # 태그 필터링 (라인에서 태그 부분을 추출해야 함, brief 포맷 기준)
                # D/MyTag ( 1234): Log message
                tag_part = ""
                if "/" in line and "(" in line:
                    try:
                        tag_part = line.split("/")[1].split("(")[0].strip()
                    except IndexError:
                        pass
                
                tag_match = True
                if self.tag_filter and self.tag_filter.lower() not in tag_part.lower():
                    tag_match = False
                
                msg_match = True
                if self.msg_filter and self.msg_filter not in line_lower:
                    msg_match = False

                if tag_match and msg_match:
                    self.new_log_line.emit(line.strip())
            
            stream.close() # adbutils 스트림은 close 필요

        except FileNotFoundError:
            self.log_message.emit(f"LogcatWorker: 'adb' command not found. Ensure ADB is in your PATH.")
        except Exception as e:
            self.log_message.emit(f"LogcatWorker: Error reading logcat stream: {e}")
        finally:
            if self.process:
                self.process.terminate()
            self.log_message.emit("LogcatWorker: Stopped.")
            self._is_running = False


    def stop(self):
        self._is_running = False
        if self.process:
            self.process.terminate() # SIGTERM
            # 필요시 self.process.kill() # SIGKILL


class MainWindow(QMainWindow):
    def __init__(
        self,
        max_width: Optional[int],
        serial: Optional[str] = None,
    ):
        super().__init__()
        self.setWindowTitle("AI Android Automation")
        self.max_width = max_width
        self.current_adb_serial = serial
        self.current_frame_pil_bytes = None # 현재 화면 PIL 이미지의 바이트 데이터 (PNG)
        self.current_ui_elements: List[Dict] = [] # 현재 UI 덤프 요소 저장
        self.original_screen_size = None # scrcpy 첫 프레임 크기
        self.device_native_resolution = None # ADB로 가져온 실제 단말 해상도

        # --- AI Model ---
        self.clip_model = FastVlmModelWrapper() # 새 Wrapper 사용
        if not self.clip_model.is_ready:
            self.log_dev_message("Error: AI model (FastVLM conceptual) could not be loaded.")


        # --- UI Setup ---
        self.setup_ui()

        # --- ADB & Scrcpy Setup ---
        self.devices = self.list_devices()
        if self.current_adb_serial:
            if self.current_adb_serial not in self.devices:
                QMessageBox.warning(self, "Device Error", f"Device serial [{self.current_adb_serial}] not found!")
                self.current_adb_serial = self.combo_device.currentText() if self.devices else None
            else:
                self.combo_device.setCurrentText(self.current_adb_serial)

        self.adb_device = None # adbutils device instance
        self.client = None    # scrcpy client instance
        self.alive = True

        if self.current_adb_serial:
            self.connect_to_device(self.current_adb_serial)
        else:
            self.log_dev_message("No device selected or found on startup.")


        # --- Logcat Worker ---
        self.logcat_worker = None

        # --- Bindings ---
        self.combo_device.currentTextChanged.connect(self.on_device_changed)
        self.button_dump_ui.clicked.connect(self.fetch_and_show_ui_dump) # 새 버튼에 연결
        # Mouse/Key events for scrcpy label are setup in connect_to_device

    def setup_ui(self):
        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)
        main_layout = QHBoxLayout(self.main_widget)

        # Left: Scrcpy Screen and Overlay
        left_panel_container = QWidget() # screen_label과 overlay_widget을 담을 컨테이너
        left_panel_layout = QVBoxLayout(left_panel_container) # 실제로는 스택 레이아웃이 더 적합하나, 여기서는 QLabel 위에 QWidget을 올리는 방식
        left_panel_layout.setContentsMargins(0,0,0,0) # 여백 제거
        
        self.screen_label = QLabel("Connecting to device...")
        self.screen_label.setMinimumSize(400, 700)
        self.screen_label.setStyleSheet("background-color: black; color: white;")
        self.screen_label.setAlignment(Qt.AlignCenter)
        # self.screen_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding) # 크기 조절

        # Overlay Widget 생성 및 screen_label 위에 배치
        # QLabel은 다른 위젯을 자식으로 직접 추가하기 어렵거나 복잡해질 수 있음.
        # QLabel을 포함하는 부모 위젯을 만들고, 그 부모 위젯 내에서 QLabel과 OverlayWidget을 겹치게 배치.
        # 또는, QLabel의 paintEvent를 오버라이드 하거나, QLabel 대신 QWidget을 사용하고 직접 그리는 방법도 있음.
        # 여기서는 QLabel과 같은 크기/위치의 별도 투명 위젯으로 구현하고, MainWindow에서 위치를 동기화.
        self.overlay_widget = ScreenOverlayWidget(self.screen_label) # screen_label을 부모로 하여 위치 동기화 용이
        self.overlay_widget.setVisible(False) # 기본적으로 숨김

        left_panel_layout.addWidget(self.screen_label) # QVBoxLayout 사용 시 겹치지 않음.
                                                        # QStackedLayout이나 수동으로setGeometry 필요.
                                                        # 가장 간단하게는 overlay_widget을 screen_label의 자식으로 하고,
                                                        # screen_label의 resizeEvent에서 overlay_widget 크기 조절.

        main_layout.addWidget(left_panel_container, 2)


        # Right: Control Panel
        right_panel = QWidget()
        right_v_layout = QVBoxLayout(right_panel)
        main_layout.addWidget(right_panel, 1)

        # Device Selection
        device_box = QHBoxLayout()
        device_box.addWidget(QLabel("Device:"))
        self.combo_device = QComboBox()
        device_box.addWidget(self.combo_device)
        self.button_refresh_devices = QPushButton("Refresh")
        self.button_refresh_devices.clicked.connect(self.list_devices)
        device_box.addWidget(self.button_refresh_devices)
        right_v_layout.addLayout(device_box)

        # Command Input
        command_box = QHBoxLayout()
        self.command_input = QLineEdit()
        self.command_input.setPlaceholderText("Enter command (e.g., 'click login button')")
        self.command_input.returnPressed.connect(self.send_ai_command) # 엔터로 전송
        command_box.addWidget(self.command_input)
        self.button_send_command = QPushButton("Send")
        self.button_send_command.clicked.connect(self.send_ai_command)
        command_box.addWidget(self.button_send_command)
        right_v_layout.addLayout(command_box)

        # Status Display
        right_v_layout.addWidget(QLabel("Status:"))
        self.status_display = QTextEdit()
        self.status_display.setReadOnly(True)
        self.status_display.setFixedHeight(100)
        right_v_layout.addWidget(self.status_display)

        # Dev Logs
        right_v_layout.addWidget(QLabel("Developer Logs:"))
        self.dev_log_display = QTextEdit()
        self.dev_log_display.setReadOnly(True)
        right_v_layout.addWidget(self.dev_log_display)

        # UI Dump Controls
        ui_dump_controls_layout = QHBoxLayout()
        self.button_dump_ui = QPushButton("Fetch UI Dump") # 기존에 없던 버튼
        ui_dump_controls_layout.addWidget(self.button_dump_ui)

        self.button_toggle_overlay = QPushButton("Show UI Overlay")
        self.button_toggle_overlay.setCheckable(True)
        self.button_toggle_overlay.clicked.connect(self.toggle_ui_overlay)
        ui_dump_controls_layout.addWidget(self.button_toggle_overlay)
        right_v_layout.addLayout(ui_dump_controls_layout)
        
        # Logcat Section
        self.button_toggle_logcat = QPushButton("Show Logcat (+)")
        self.button_toggle_logcat.setCheckable(True)
        self.button_toggle_logcat.clicked.connect(self.toggle_logcat_panel)
        right_v_layout.addWidget(self.button_toggle_logcat)

        self.logcat_panel = QGroupBox("Logcat")
        logcat_layout = QVBoxLayout(self.logcat_panel)

        logcat_filters_layout = QFormLayout()
        self.logcat_tag_filter = QLineEdit()
        self.logcat_tag_filter.setPlaceholderText("e.g., MyApp")
        logcat_filters_layout.addRow("Tag Filter:", self.logcat_tag_filter)
        self.logcat_msg_filter = QLineEdit()
        self.logcat_msg_filter.setPlaceholderText("e.g., error, clicked")
        logcat_filters_layout.addRow("Message Filter:", self.logcat_msg_filter)
        logcat_layout.addLayout(logcat_filters_layout)
        
        self.button_start_logcat = QPushButton("Start/Update Logcat")
        self.button_start_logcat.clicked.connect(self.start_or_update_logcat)
        logcat_layout.addWidget(self.button_start_logcat)

        self.logcat_display = QTextEdit()
        self.logcat_display.setReadOnly(True)
        self.logcat_display.setFontFamily("Monospace") # 고정폭 폰트
        logcat_layout.addWidget(self.logcat_display)

        self.logcat_panel.setVisible(False) # 기본적으로 숨김
        right_v_layout.addWidget(self.logcat_panel)
        
        right_v_layout.addStretch() # 하단 공간 채우기
        self.screen_label.installEventFilter(self) # screen_label의 이벤트를 MainWindow에서 감지

    def eventFilter(self, watched_object, event):
        # screen_label의 크기가 변경될 때 overlay_widget의 크기도 동일하게 설정
        if watched_object == self.screen_label and event.type() == QEvent.Resize:
            self.overlay_widget.resize(self.screen_label.size())
        return super().eventFilter(watched_object, event)
    
    def connect_to_device(self, serial: str):
        self.log_dev_message(f"Attempting to connect to device: {serial}")
        self.current_adb_serial = serial
        try:
            self.adb_device = adb.device(serial=serial)
            self.log_dev_message(f"ADB connected to: {self.adb_device.serial}")

            # 단말기 실제 해상도 가져오기
            try:
                size_str = self.adb_device.shell("wm size") 
                if size_str and "Physical size:" in size_str:
                    res_part = size_str.split("Physical size:")[1].strip()
                    w_str, h_str = res_part.split("x")
                    self.device_native_resolution = (int(w_str), int(h_str))
                    self.log_dev_message(f"Fetched native device resolution: {self.device_native_resolution}")
                else:
                    self.log_dev_message(f"Could not parse native device resolution from 'wm size': {size_str}")
                    self.device_native_resolution = None 
            except Exception as e:
                self.log_dev_message(f"Error fetching native device resolution: {e}")
                self.device_native_resolution = None

            if self.client:
                self.client.stop()
                self.client.remove_all_listeners()
            self.client = scrcpy.Client(
                device=self.adb_device,
                max_width=self.max_width, 
                bitrate=8000000, 
            )
            self.client.add_listener(scrcpy.EVENT_INIT, self.on_init_scrcpy)
            self.client.add_listener(scrcpy.EVENT_FRAME, self.on_frame_scrcpy)
            self.client.add_listener(scrcpy.EVENT_DISCONNECT, self.on_disconnect_scrcpy)
            
            self.screen_label.mousePressEvent = self._create_mouse_event_handler(scrcpy.ACTION_DOWN)
            self.screen_label.mouseMoveEvent = self._create_mouse_event_handler(scrcpy.ACTION_MOVE)
            self.screen_label.mouseReleaseEvent = self._create_mouse_event_handler(scrcpy.ACTION_UP)
            
            self.log_dev_message("Starting scrcpy client...")
            self.client.start(threaded=True) 
            self.log_status_message(f"Connecting to {serial}...")

        except AdbError as e:
            self.log_dev_message(f"ADB Error connecting to {serial}: {e}")
            QMessageBox.critical(self, "ADB Error", f"Could not connect to device {serial}:\\n{e}")
            self.screen_label.setText(f"Failed to connect to {serial}.\\nCheck ADB connection.")
        except Exception as e:
            self.log_dev_message(f"General Error connecting to {serial}: {e}")
            QMessageBox.critical(self, "Connection Error", f"An unexpected error occurred with device {serial}:\\n{e}")
            self.screen_label.setText("Connection error.")

    def on_device_changed(self, serial: str):
        if serial and serial != self.current_adb_serial:
            self.screen_label.setText(f"Connecting to {serial}...")
            QApplication.processEvents() # UI 업데이트 강제
            if self.logcat_worker and self.logcat_worker.isRunning():
                self.logcat_worker.stop()
                self.logcat_worker.wait() # 스레드 종료 대기
            self.connect_to_device(serial)


    def list_devices(self) -> List[str]:
        self.combo_device.clear()
        try:
            items = [d.serial for d in adb.device_list()]
            if items:
                self.combo_device.addItems(items)
                if not self.current_adb_serial or self.current_adb_serial not in items:
                    self.current_adb_serial = items[0]
                self.combo_device.setCurrentText(self.current_adb_serial)
                self.log_dev_message(f"Devices found: {items}")
            else:
                self.log_dev_message("No ADB devices found.")
                self.current_adb_serial = None
                self.screen_label.setText("No ADB devices found. Please connect a device and refresh.")
            return items
        except AdbError as e:
            self.log_dev_message(f"ADB Error listing devices: {e}")
            QMessageBox.warning(self, "ADB Error", "Could not list ADB devices. Is ADB running and configured?")
            self.screen_label.setText("Error listing ADB devices.")
            return []


    def on_init_scrcpy(self):
        if self.client and self.client.device_name:
            self.setWindowTitle(f"AI Android Automation - {self.client.device_name}")
            self.log_status_message(f"Connected to {self.client.device_name}")
            self.log_dev_message(f"scrcpy initialized for {self.client.device_name}. Resolution: {self.client.resolution}")
        else:
            self.log_dev_message("scrcpy initialized, but client or device_name is None.")


    def on_frame_scrcpy(self, frame):
        if frame is None:
            return

        # 원본 화면 크기 저장 (첫 프레임에서만)
        if self.original_screen_size is None:
            self.original_screen_size = (frame.shape[1], frame.shape[0])

        # BGR to RGB 변환 및 연속된 메모리 공간 확보
        rgb_frame = frame[:, :, ::-1].copy()  # copy()를 사용하여 연속된 메모리 공간 확보
        
        # PIL Image로 변환
        pil_image = Image.fromarray(rgb_frame)
        
        # QImage로 변환 (RGB 형식으로)
        height, width, channel = frame.shape
        bytes_per_line = 3 * width
        q_image = QImage(rgb_frame.data, width, height, bytes_per_line, QImage.Format_RGB888)
        
        # QPixmap으로 변환
        current_pixmap = QPixmap.fromImage(q_image) # 변수명 변경
        
        # 최대 너비에 맞게 크기 조정
        if self.max_width and current_pixmap.width() > self.max_width:
            current_pixmap = current_pixmap.scaledToWidth(self.max_width, Qt.SmoothTransformation)
        
        # 화면 업데이트
        self.screen_label.setPixmap(current_pixmap)
        
        # 현재 화면 이미지 저장 (AI 명령 처리용)
        self.current_screen_image = pil_image
        
        # PIL 이미지를 바이트로 변환하여 저장
        img_byte_arr = io.BytesIO()
        pil_image.save(img_byte_arr, format='PNG')
        self.current_frame_pil_bytes = img_byte_arr.getvalue()

        # 오버레이 위젯 지오메트리 정보 업데이트
        if self.overlay_widget and self.screen_label.pixmap():
            final_pixmap = self.screen_label.pixmap() # screen_label에 실제로 설정된 pixmap
            pix_w = final_pixmap.width()
            pix_h = final_pixmap.height()
            
            label_w = self.screen_label.width()
            label_h = self.screen_label.height()
            
            # QLabel의 alignment가 Qt.AlignCenter라고 가정하고 오프셋 계산
            offset_x = (label_w - pix_w) / 2.0
            offset_y = (label_h - pix_h) / 2.0
            
            # overlay_widget에 전달되는 좌표는 이미 pixmap 기준으로 스케일링되었으므로 scale은 1.0
            self.overlay_widget.update_geometry_info(1.0, offset_x, offset_y, pix_w, pix_h)


    def on_disconnect_scrcpy(self):
        self.log_status_message("Device disconnected.")
        self.log_dev_message(f"scrcpy client for {self.current_adb_serial} disconnected.")
        self.screen_label.setText(f"Device {self.current_adb_serial} disconnected.\nSelect a device or refresh.")
        # 자동 재연결 로직 추가 가능


    def send_ai_command(self):
        command = self.command_input.text().strip()
        if not command:
            self.log_status_message("Please enter a command.")
            return
        if not self.clip_model or not self.clip_model.is_ready:
            self.log_status_message("AI model is not ready.")
            QMessageBox.warning(self, "AI Error", "FastVLM model is not loaded or ready.")
            return
        if not self.current_adb_serial:
            self.log_status_message("No device connected.")
            QMessageBox.warning(self, "Device Error", "Please connect to an Android device first.")
            return
        if not hasattr(self, 'current_screen_image') or self.current_screen_image is None:
            self.log_status_message("No screen capture available.")
            QMessageBox.warning(self, "Screen Error", "Could not get current screen image.")
            return

        self.log_dev_message(f"User command: {command}")
        self.command_input.clear()

        # AI 처리를 위한 스레드 시작
        self.ai_worker = AICommandWorker(self.clip_model, self.current_adb_serial, self.current_frame_pil_bytes, command)
        self.ai_worker.status_update.connect(self.log_status_message)
        self.ai_worker.log_message.connect(self.log_dev_message)
        self.ai_worker.action_result.connect(self.on_ai_action_success)
        self.ai_worker.action_failed.connect(self.on_ai_action_failed)
        self.ai_worker.start()

    @Slot(str, int, int)
    def on_ai_action_success(self, message: str, x: int, y: int):
        self.log_status_message(message)
        self.log_dev_message(f"AI action success reported: {message} (Original coords: {x}, {y})")
        
        # AICommandWorker가 adbutils로 실제 액션을 수행하므로 아래 scrcpy 코드는 주석 처리 또는 제거
        # if self.client and self.client.control:
        #     try:
        #         # scrcpy의 좌표계는 화면 해상도 기준. UI Dump의 좌표도 화면 해상도 기준이므로 변환 불필요.
        #         self.client.control.touch(x, y, scrcpy.ACTION_DOWN)
        #         self.client.control.touch(x, y, scrcpy.ACTION_UP)
        #         self.log_dev_message(f"Clicked at ({x},{y}) via scrcpy.") # 이 부분은 이제 AIWorker가 처리
        #     except Exception as e:
        #         self.log_dev_message(f"Error during scrcpy click: {e}")
        #         self.log_status_message(f"Click failed: {e}")
        # else:
        #     self.log_dev_message("scrcpy client or control not available for click.")
        #     self.log_status_message("Click failed: scrcpy client not ready.")

    @Slot(str)
    def on_ai_action_failed(self, message: str):
        self.log_status_message(f"AI Action Failed: {message}")
        self.log_dev_message(f"AI Action Failed: {message}")

    @Slot()
    def fetch_and_show_ui_dump(self):
        if not self.current_adb_serial:
            self.log_status_message("No device connected")
            return False # 실패 반환

        try:
            xml_content = get_ui_dump(self.current_adb_serial)
            if not xml_content:
                self.log_status_message("Failed to get UI dump")
                return False

            elements = parse_ui_dump(xml_content) # bounds_rect는 원본 해상도 기준
            if not elements:
                self.log_status_message("No UI elements found")
                return False

            source_width, source_height = -1, -1
            source_res_info = "Unknown"
            if self.device_native_resolution:
                source_width, source_height = self.device_native_resolution
                source_res_info = f"NATIVE ({source_width}x{source_height})"
            elif self.original_screen_size: # scrcpy 첫 프레임 해상도
                source_width, source_height = self.original_screen_size
                source_res_info = f"Scrcpy Frame ({source_width}x{source_height})"
            
            if source_width <= 0 or source_height <= 0:
                self.log_status_message("Cannot determine source resolution for UI dump scaling.")
                self.log_dev_message("UI Dump Scaling Error: Source resolution invalid.")
                self.current_ui_elements = []
                return False

            self.log_dev_message(f"UI Dump Scaling: Using source resolution {source_res_info}")

            # 현재 화면에 표시된 QPixmap 정보 가져오기
            current_display_pixmap = self.screen_label.pixmap()
            if not current_display_pixmap or current_display_pixmap.isNull():
                self.log_status_message("Screen pixmap not available for UI dump scaling.")
                self.log_dev_message("UI Dump Scaling Error: Target pixmap invalid.")
                self.current_ui_elements = []
                return False

            target_width = current_display_pixmap.width()
            target_height = current_display_pixmap.height()
            self.log_dev_message(f"UI Dump Scaling: Target pixmap resolution ({target_width}x{target_height})")


            scale_x = target_width / source_width if source_width > 0 else 1.0
            scale_y = target_height / source_height if source_height > 0 else 1.0
            self.log_dev_message(f"UI Dump Scaling: Scale factors ({scale_x:.2f}, {scale_y:.2f})")

            processed_elements = []
            for element in elements:
                bounds_rect = element.get("bounds_rect") # 원본 (x, y, w, h)
                if bounds_rect:
                    x, y, w, h = bounds_rect
                    # QPixmap 기준으로 스케일링된 좌표
                    scaled_px = int(x * scale_x)
                    scaled_py = int(y * scale_y)
                    scaled_pw = int(w * scale_x)
                    scaled_ph = int(h * scale_y)
                    
                    # 스케일링 결과가 너무 작으면 (예: 0x0) 건너뛰거나 최소 크기 보장
                    if scaled_pw < 1 : scaled_pw = 1
                    if scaled_ph < 1 : scaled_ph = 1

                    element["scaled_bounds_rect_for_pixmap"] = (scaled_px, scaled_py, scaled_pw, scaled_ph)
                processed_elements.append(element)
            
            self.current_ui_elements = processed_elements
            self.overlay_widget.set_ui_elements(self.current_ui_elements)
            self.log_status_message(f"Found {len(self.current_ui_elements)} UI elements, scaled for display.")
            return True
        except Exception as e:
            self.log_dev_message(f"Error in fetch_and_show_ui_dump: {str(e)}")
            self.log_status_message(f"Failed to load UI dump: {e}")
            self.current_ui_elements = []
            return False
            
    @Slot()
    def toggle_ui_overlay(self):
        if self.button_toggle_overlay.isChecked():
            if not hasattr(self, 'current_screen_image') or self.current_screen_image is None:
                self.log_status_message("Screen image not available. Please wait for the screen to be captured.")
                self.button_toggle_overlay.setChecked(False)
                return
                
            if not self.current_ui_elements: # UI Dump 데이터가 없으면 먼저 가져오기
                self.log_status_message("UI Dump data not loaded. Fetching first...")
                if not self.fetch_and_show_ui_dump(): # dump를 가져오고 성공 여부 확인
                    self.button_toggle_overlay.setChecked(False) # 실패시 버튼 상태 원복
                    return

            self.overlay_widget.set_ui_elements(self.current_ui_elements)
            self.overlay_widget.setVisible(True)
            self.button_toggle_overlay.setText("Hide UI Overlay")
            self.log_dev_message("UI Overlay shown.")
        else:
            self.overlay_widget.setVisible(False)
            self.button_toggle_overlay.setText("Show UI Overlay")
            QToolTip.hideText() # 오버레이 숨길 때 툴큅도 숨김
            self.log_dev_message("UI Overlay hidden.")
            
    def toggle_logcat_panel(self):
        if self.button_toggle_logcat.isChecked():
            self.logcat_panel.setVisible(True)
            self.button_toggle_logcat.setText("Hide Logcat (-)")
            # 처음 열 때 자동으로 Logcat 시작 (선택 사항)
            # if not self.logcat_worker or not self.logcat_worker.isRunning():
            #    self.start_or_update_logcat()
        else:
            self.logcat_panel.setVisible(False)
            self.button_toggle_logcat.setText("Show Logcat (+)")
            if self.logcat_worker and self.logcat_worker.isRunning():
                self.logcat_worker.stop()


    def start_or_update_logcat(self):
        if not self.current_adb_serial:
            QMessageBox.warning(self, "Device Error", "No device selected for Logcat.")
            return

        if self.logcat_worker and self.logcat_worker.isRunning():
            self.logcat_worker.stop()
            self.logcat_worker.wait() # 이전 스레드 종료 확실히 대기

        tag_filter = self.logcat_tag_filter.text()
        msg_filter = self.logcat_msg_filter.text()
        
        self.logcat_display.clear() # 새 필터 적용 시 이전 로그 지우기
        self.log_dev_message(f"Starting Logcat for {self.current_adb_serial}. Tag: '{tag_filter}', Msg: '{msg_filter}'")
        
        self.logcat_worker = LogcatWorker(self.current_adb_serial, tag_filter, msg_filter)
        self.logcat_worker.new_log_line.connect(self.append_logcat_message)
        self.logcat_worker.log_message.connect(self.log_dev_message) # LogcatWorker 자체 로그
        self.logcat_worker.start()


    @Slot(str)
    def append_logcat_message(self, line: str):
        self.logcat_display.append(line)
        # 자동 스크롤 (너무 많은 로그가 빠르게 들어오면 성능에 영향 줄 수 있음)
        # self.logcat_display.verticalScrollBar().setValue(self.logcat_display.verticalScrollBar().maximum())


    @Slot(str)
    def log_status_message(self, message: str):
        self.status_display.append(message)

    @Slot(str)
    def log_dev_message(self, message: str):
        self.dev_log_display.append(message)


    def _create_mouse_event_handler(self, action: int):
        def handler(evt: QMouseEvent):
            if self.client and self.client.control and self.client.resolution:
                # QLabel 좌표를 scrcpy 실제 화면 좌표로 변환
                # 현재 on_frame에서 pixmap을 label 크기에 맞게 스케일링하므로,
                # label 상의 마우스 좌표가 곧 스케일링된 이미지 상의 좌표.
                # 이를 원래 해상도 좌표로 역변환해야 함.

                label_size = self.screen_label.size()
                pixmap_size = self.screen_label.pixmap().size() # 실제 그려진 pixmap 크기

                if pixmap_size.isEmpty() or label_size.isEmpty(): return

                # Pixmap이 Label 내에서 어떻게 배치되었는지 (비율 유지 스케일링 고려)
                # (Label 중앙에 그려짐)
                label_w, label_h = label_size.width(), label_size.height()
                pix_w, pix_h = pixmap_size.width(), pixmap_size.height()
                
                offset_x = (label_w - pix_w) / 2
                offset_y = (label_h - pix_h) / 2

                mouse_x_on_pixmap = evt.position().x() - offset_x
                mouse_y_on_pixmap = evt.position().y() - offset_y
                
                # Pixmap 상 좌표가 유효 범위 내인지 확인
                if not (0 <= mouse_x_on_pixmap <= pix_w and 0 <= mouse_y_on_pixmap <= pix_h):
                    return

                # Pixmap 좌표를 원본 해상도 좌표로 변환
                # client.resolution은 (width, height)
                original_w, original_h = self.client.resolution
                
                # 스케일링 비율은 pix_w / original_w 또는 pix_h / original_h 중 하나 사용
                # (KeepAspectRatio 이므로 같아야 하지만, 부동소수점 오차 가능성)
                scale_w = pix_w / original_w
                scale_h = pix_h / original_h
                
                # 더 작은 스케일 값을 사용 (비율 유지 스케일링 시 빈 공간이 생기는 축 기준)
                # 더 정확하게는, on_frame에서 어떤 기준으로 스케일링 했는지 알아야 함
                # 여기서는 scrcpy가 max_width에 맞춰서 보내주고, label에 그대로 표시한다고 가정
                # 이 경우, label의 pixmap은 scrcpy에서 보내준 프레임과 동일하거나 축소된 버전
                # self.client.resolution이 실제 디바이스 해상도
                # self.screen_label.pixmap().size()가 현재 표시되는 픽스맵 크기
                # max_width (또는 max_height) 기준으로 scrcpy가 프레임을 스케일링
                # client.resolution은 원본 해상도일 것.
                # scrcpy Client 생성시 max_width, max_height 등으로 프레임 크기 조절.
                # 프레임 크기가 (frame_w, frame_h)이고, 마우스 클릭이 (mouse_x, mouse_y)이면
                # 실제 터치 좌표는 (mouse_x * original_w / frame_w, mouse_y * original_h / frame_h)

                # on_frame에서 QImage 만들 때의 w,h 가 scrcpy가 보내준 프레임의 실제 크기
                # current_frame_pil_bytes 만들 때 사용한 pil_img.size가 프레임 크기
                if self.current_frame_pil_bytes:
                    try:
                        temp_pil = Image.open(io.BytesIO(self.current_frame_pil_bytes))
                        frame_w, frame_h = temp_pil.size
                        
                        # 마우스 좌표는 pixmap 기준. pixmap은 frame_w, frame_h와 같아야 함(UI 스케일링 없을 시)
                        # 여기서는 pixmap_size가 frame_w, frame_h와 같다고 가정
                        # (label.size()와 pixmap.size()가 다를 수 있음에 유의)
                        # 마우스 클릭은 pixmap 영역 안에서만 유효하게 처리

                        target_x = mouse_x_on_pixmap * (original_w / pix_w)
                        target_y = mouse_y_on_pixmap * (original_h / pix_h)

                        self.client.control.touch(int(target_x), int(target_y), action)
                    except Exception as e:
                        self.log_dev_message(f"Mouse event coordinate calculation error: {e}")
                
            # 키보드 포커스 해제 (LineEdit 등에 포커스 되어 있으면 키 이벤트가 거기로 감)
            focused_widget = QApplication.focusWidget()
            if focused_widget is not None and isinstance(focused_widget, QLineEdit):
                focused_widget.clearFocus()
        return handler


    # 키 이벤트는 MainWindow 레벨에서 처리
    def keyPressEvent(self, event: QKeyEvent):
        if self.client and self.client.control:
            # 입력 필드에 포커스가 있을 때는 키 이벤트를 scrcpy로 보내지 않음
            if QApplication.focusWidget() not in [self.command_input, self.logcat_tag_filter, self.logcat_msg_filter]:
                keycode = self._map_qt_key_to_android(event.key(), event.text())
                if keycode != -1:
                    self.client.control.keycode(keycode, scrcpy.ACTION_DOWN)
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event: QKeyEvent):
        if self.client and self.client.control:
            if QApplication.focusWidget() not in [self.command_input, self.logcat_tag_filter, self.logcat_msg_filter]:
                keycode = self._map_qt_key_to_android(event.key(), event.text())
                if keycode != -1:
                    self.client.control.keycode(keycode, scrcpy.ACTION_UP)
        super().keyReleaseEvent(event)

    def _map_qt_key_to_android(self, qt_key_code, text=""): # text 인자 추가 for 문자키
        # 숫자 (0-9)
        if Qt.Key_0 <= qt_key_code <= Qt.Key_9:
            return qt_key_code - Qt.Key_0 + scrcpy.KEYCODE_0
        # 알파벳 (A-Z) - Qt는 대문자만 반환, scrcpy는 대소문자 구분 없음
        if Qt.Key_A <= qt_key_code <= Qt.Key_Z:
            return qt_key_code - Qt.Key_A + scrcpy.KEYCODE_A

        # 특수키 매핑 (더 많은 키 추가 가능)
        # scrcpy.consts 참조
        key_map = {
            Qt.Key_Space: scrcpy.KEYCODE_SPACE,
            Qt.Key_Enter: scrcpy.KEYCODE_ENTER,
            Qt.Key_Return: scrcpy.KEYCODE_ENTER, # Enter와 Return 동일 취급
            Qt.Key_Backspace: scrcpy.KEYCODE_DEL, # Backspace -> DEL
            Qt.Key_Delete: scrcpy.KEYCODE_FORWARD_DEL, # Delete -> FORWARD_DEL
            Qt.Key_Tab: scrcpy.KEYCODE_TAB,
            Qt.Key_Escape: scrcpy.KEYCODE_ESCAPE,
            Qt.Key_Home: scrcpy.KEYCODE_MOVE_HOME,
            Qt.Key_End: scrcpy.KEYCODE_MOVE_END,
            Qt.Key_Left: scrcpy.KEYCODE_DPAD_LEFT,
            Qt.Key_Right: scrcpy.KEYCODE_DPAD_RIGHT,
            Qt.Key_Up: scrcpy.KEYCODE_DPAD_UP,
            Qt.Key_Down: scrcpy.KEYCODE_DPAD_DOWN,
            Qt.Key_Shift: scrcpy.KEYCODE_SHIFT_LEFT,
            Qt.Key_Control: scrcpy.KEYCODE_CTRL_LEFT,
            Qt.Key_Alt: scrcpy.KEYCODE_ALT_LEFT,
            # TODO: 더 많은 키들... 예: 심볼, F1-F12 등
        }
        if qt_key_code in key_map:
            return key_map[qt_key_code]
        
        # text 인자를 사용한 문자 처리 (위에서 처리 안된 심볼 등)
        # 이 부분은 안드로이드 키코드로 정확히 매핑하기 어려울 수 있음
        # scrcpy control.text() 사용이 더 나을 수 있음 (키코드 대신)
        # if text:
        #     self.log_dev_message(f"Sending text input: {text}")
        #     if self.client and self.client.control:
        #         self.client.control.text(text) # 텍스트 직접 입력
        #     return -2 # 텍스트 전송했으므로 키코드 전송 안함

        self.log_dev_message(f"Unhandled Qt Key: {qt_key_code}, Text: '{text}'")
        return -1 # 매핑되지 않은 키


    def closeEvent(self, event):
        self.log_dev_message("Closing application...")
        self.alive = False
        if self.logcat_worker and self.logcat_worker.isRunning():
            self.logcat_worker.stop()
            self.logcat_worker.wait()
        if self.client:
            self.client.stop()
            # scrcpy client 내부 스레드가 완전히 종료될 때까지 잠시 대기할 수 있음
            # (필요시 self.client.join() 같은 명시적 종료 대기 메서드 확인)
        
        # AI 스레드가 실행 중이면 종료 시도
        if hasattr(self, 'ai_worker') and self.ai_worker and self.ai_worker.isRunning():
            self.log_dev_message("Waiting for AI worker to finish...")
            # AI 스레드는 중간에 강제 종료하기 어려우므로, 완료될 때까지 기다리거나
            # AI 스레드 내부에 주기적으로 self.alive 같은 플래그를 확인하는 로직 추가 필요
            self.ai_worker.wait(2000) # 최대 2초 대기

        super().closeEvent(event)


def main():
    if not QApplication.instance():
        app = QApplication(sys.argv)
    else:
        app = QApplication.instance()

    parser = ArgumentParser(description="AI Android Automation Tool")
    parser.add_argument(
        "-m", "--max_width", type=int, default=800,
        help="Max width for scrcpy video stream (rendering quality)"
    )
    parser.add_argument(
        "-d", "--device", type=str, help="Device serial to connect to initially"
    )
    # encoder_name 인자는 scrcpy client 생성 시 사용 가능
    # parser.add_argument("--encoder_name", type=str, help="Encoder name for scrcpy")
    args = parser.parse_args()

    main_window = MainWindow(max_width=args.max_width, serial=args.device)
    main_window.resize(1200, 800) # 윈도우 초기 크기
    main_window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()