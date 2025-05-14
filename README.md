# AutoGUI - AI-Powered Android Automation Tool

AutoGUI is a powerful desktop application that combines AI capabilities with Android device control. It allows you to automate Android device interactions using natural language commands and provides real-time device screen mirroring.

## Features

- Real-time Android screen mirroring using scrcpy
- AI-powered natural language command processing
- UI element detection and interaction
- Real-time logcat monitoring
- Interactive UI overlay for element inspection
- Direct touch and keyboard input support

## Prerequisites

- Python 3.8 or higher
- ADB (Android Debug Bridge) installed and in PATH
- Android device with USB debugging enabled
- USB connection to Android device

## Installation

1. Clone the repository:
```bash
git clone https://github.com/okdoggy/autoGUI.git
cd autoGUI
```

2. Install uv (Python package manager):
```bash
# Windows (PowerShell)
curl -sSf https://astral.sh/uv/install.ps1 | powershell

# Linux/macOS
curl -sSf https://astral.sh/uv/install.sh | sh
```

3. Create and activate a virtual environment using uv:
```bash
uv venv
# Windows
.venv\Scripts\activate
# Linux/macOS
source .venv/bin/activate
```

4. Install dependencies using uv:
```bash
uv pip install -r requirements.txt
```

## Usage

1. Connect your Android device via USB and enable USB debugging

2. Run the application:
```bash
python main.py
```

3. Select your device from the dropdown menu if not automatically detected

4. Use the command input field to enter natural language commands like:
   - "Click the login button"
   - "Type 'hello world' in the search box"
   - "Find the settings menu"

5. Use the UI overlay to inspect elements on the screen

6. Monitor device logs using the logcat panel

## Command Examples

- "Click the login button"
- "Type 'hello world' in the search box"
- "Find and click the settings icon"
- "Scroll down to the bottom"
- "Type 'test@example.com' in the email field"

## Development

The project uses:
- PySide6 for the GUI
- scrcpy for screen mirroring
- adbutils for ADB communication
- PIL for image processing

## License

MIT License

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request. 