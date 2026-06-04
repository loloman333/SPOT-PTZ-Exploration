import sys
import subprocess
import os
from datetime import datetime
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
    QInputDialog, QFileDialog, QMessageBox, QLabel, QLineEdit, QSizePolicy
)
from PyQt5.QtCore import Qt, QProcess
import yaml # Import the PyYAML library
from ament_index_python.packages import get_package_share_directory

# Define a custom QMessageBox class for consistent styling
class CustomMessageBox(QMessageBox):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet("font-size: 18px;") # Apply default styling for all instances

    @staticmethod
    def information(parent, title, text, buttons=QMessageBox.Ok, font_size="18px"):
        msg_box = CustomMessageBox(parent)
        msg_box.setWindowTitle(title)
        msg_box.setText(text)
        msg_box.setIcon(QMessageBox.Information)
        msg_box.setStandardButtons(buttons)
        msg_box.setStyleSheet(f"font-size: {font_size};") # Allow overriding font size
        return msg_box.exec_()

    @staticmethod
    def warning(parent, title, text, buttons=QMessageBox.Ok, font_size="18px"):
        msg_box = CustomMessageBox(parent)
        msg_box.setWindowTitle(title)
        msg_box.setText(text)
        msg_box.setIcon(QMessageBox.Warning)
        msg_box.setStandardButtons(buttons)
        msg_box.setStyleSheet(f"font-size: {font_size};")
        return msg_box.exec_()

    @staticmethod
    def critical(parent, title, text, buttons=QMessageBox.Ok, font_size="18px"):
        msg_box = CustomMessageBox(parent)
        msg_box.setWindowTitle(title)
        msg_box.setText(text)
        msg_box.setIcon(QMessageBox.Critical)
        msg_box.setStandardButtons(buttons)
        msg_box.setStyleSheet(f"font-size: {font_size};")
        return msg_box.exec_()

    @staticmethod
    def question(parent, title, text, buttons=QMessageBox.Ok, font_size="18px"):
        msg_box = CustomMessageBox(parent)
        msg_box.setWindowTitle(title)
        msg_box.setText(text)
        msg_box.setIcon(QMessageBox.Question)
        msg_box.setStandardButtons(buttons)
        msg_box.setStyleSheet(f"font-size: {font_size};")
        return msg_box.exec_()


class Ros2BagRecorderApp(QWidget):
    def __init__(self):
        super().__init__()
        self.ros_process = None  # To hold the subprocess for ros2 bag record
        # Changed default config path to .yaml
        pkg_share = get_package_share_directory('spot_toolkit')
        self.default_config_path = os.path.join(pkg_share, 'config', 'bag_topics.yaml')
        self.current_output_folder = None # Store the output folder for renaming
        self.current_temp_bag_name = None # Store the temporary bag name for renaming
        self.initUI()

    def initUI(self):
        """Initializes the user interface."""
        self.setWindowTitle('ROS2 Bag Recorder')
        self.setGeometry(100, 100, 500, 450) # Adjusted initial window size for new widgets

        self.layout = QVBoxLayout()
        self.layout.setAlignment(Qt.AlignCenter) # Center align content

        # Information Label
        self.info_label = QLabel("Configure paths and click 'Record' to start recording ROS2 bags.")
        self.info_label.setAlignment(Qt.AlignCenter)
        self.info_label.setStyleSheet("font-size: 16px; margin-bottom: 20px;")
        self.layout.addWidget(self.info_label)

        # Output Folder Selection Widgets
        output_folder_layout = QHBoxLayout()
        self.output_folder_label = QLabel("Output Folder:")
        self.output_folder_label.setStyleSheet("font-weight: bold;")
        self.output_folder_line_edit = QLineEdit()
        self.output_folder_line_edit.setPlaceholderText("Select directory for bag files")
        self.output_folder_line_edit.setReadOnly(True) # Make it read-only as it's set by dialog
        self.output_folder_button = QPushButton("Browse...")
        self.output_folder_button.clicked.connect(self.select_output_folder)
        output_folder_layout.addWidget(self.output_folder_label)
        output_folder_layout.addWidget(self.output_folder_line_edit)
        output_folder_layout.addWidget(self.output_folder_button)
        self.layout.addLayout(output_folder_layout)

        # Config File Selection Widgets
        config_file_layout = QHBoxLayout()
        self.config_file_label = QLabel("Config File:")
        self.config_file_label.setStyleSheet("font-weight: bold;")
        self.config_file_line_edit = QLineEdit()
        self.config_file_line_edit.setPlaceholderText("Select topics config file")
        self.config_file_line_edit.setReadOnly(True) # Make it read-only
        self.config_file_line_edit.setText(self.default_config_path) # Set default path
        self.config_file_button = QPushButton("Browse...")
        self.config_file_button.clicked.connect(self.select_config_file)
        config_file_layout.addWidget(self.config_file_label)
        config_file_layout.addWidget(self.config_file_line_edit)
        config_file_layout.addWidget(self.config_file_button)
        self.layout.addLayout(config_file_layout)

        # Add some spacing
        self.layout.addSpacing(20)

        # Record Button
        self.record_button = QPushButton('Record Bag')
        # Removed setFixedSize, added setMinimumSize to allow stretching
        self.record_button.setMinimumSize(200, 80)
        self.record_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.record_button.setStyleSheet(
            "QPushButton {"
            "   background-color: #4CAF50;" # Green
            "   color: white;"
            "   font-size: 24px;"
            "   font-weight: bold;"
            "   border-radius: 15px;"
            "   border: 2px solid #388E3C;"
            "}"
            "QPushButton:hover {"
            "   background-color: #45a049;"
            "}"
            "QPushButton:pressed {"
            "   background-color: #388E3C;"
            "}"
            "QPushButton:disabled {"
            "   background-color: #cccccc;"
            "   color: #666666;"
            "   border: 2px solid #999999;"
            "}"
        )
        self.record_button.clicked.connect(self.start_recording)
        # Added stretch factor to make the button expand horizontally
        self.layout.addWidget(self.record_button, stretch=1)

        # Stop Button
        self.stop_button = QPushButton('Stop Recording')
        # Removed setFixedSize, added setMinimumSize to allow stretching
        self.stop_button.setMinimumSize(200, 80)
        self.stop_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.stop_button.setStyleSheet(
            "QPushButton {"
            "   background-color: #f44336;" # Red
            "   color: white;"
            "   font-size: 24px;"
            "   font-weight: bold;"
            "   border-radius: 15px;"
            "   border: 2px solid #d32f2f;"
            "}"
            "QPushButton:hover {"
            "   background-color: #e53935;"
            "}"
            "QPushButton:pressed {"
            "   background-color: #d32f2f;"
            "}"
            "QPushButton:disabled {"
            "   background-color: #cccccc;"
            "   color: #666666;"
            "   border: 2px solid #999999;"
            "}"
        )
        self.stop_button.clicked.connect(self.stop_recording)
        self.stop_button.setEnabled(False) # Disabled initially
        # Added stretch factor to make the button expand horizontally
        self.layout.addWidget(self.stop_button, stretch=1)

        self.setLayout(self.layout)

    def select_output_folder(self):
        """Opens a file dialog to select the output folder."""
        folder = QFileDialog.getExistingDirectory(self, 'Select Output Folder', os.getcwd())
        if folder:
            self.output_folder_line_edit.setText(folder)

    def select_config_file(self):
        """Opens a file dialog to select the topics config file."""
        file_path, _ = QFileDialog.getOpenFileName(
            self, 'Select Topics Config File', self.default_config_path, 'YAML Files (*.yaml);;All Files (*.*)'
        )
        if file_path:
            self.config_file_line_edit.setText(file_path)

    def start_recording(self):
        """Handles the logic for starting ROS2 bag recording."""

        # 1. Get output folder from QLineEdit
        output_folder = self.output_folder_line_edit.text()
        if not output_folder:
            CustomMessageBox.warning(self, 'Input Error', 'Please select an output folder.')
            return

        # 2. Get topics config file path from QLineEdit
        config_file_path = self.config_file_line_edit.text()
        if not config_file_path:
            CustomMessageBox.warning(self, 'Input Error', 'Please select a topics config file.')
            return

        topics_to_record = []
        try:
            with open(config_file_path, 'r') as f:
                config_data = yaml.safe_load(f) # Use yaml.safe_load to parse YAML

            if isinstance(config_data, dict) and 'topics' in config_data:
                topics_value = config_data['topics']
                if isinstance(topics_value, list):
                    # Filter out any non-string or empty items from the list
                    topics_to_record = [str(item).strip() for item in topics_value if isinstance(item, str) and item.strip()]
                else:
                    CustomMessageBox.warning(self, 'Config Error', 'Invalid "topics" format in YAML. Expected a list of strings or "-a".')
                    return
            elif config_data is None: # Empty YAML file
                CustomMessageBox.warning(self, 'Config Error', 'Config file is empty.')
                return
            else:
                CustomMessageBox.warning(self, 'Config Error', 'YAML file must contain a "topics" key with a list of topics or "-a".')
                return

            if not topics_to_record:
                CustomMessageBox.warning(self, 'Config Error', 'No topics found in config file after parsing.')
                return

        except yaml.YAMLError as e:
            CustomMessageBox.critical(self, 'YAML Parse Error', f'Error parsing YAML config file: {e}')
            return
        except Exception as e:
            CustomMessageBox.critical(self, 'File Error', f'Error reading config file: {e}')
            return

        # Generate temporary bag name with timestamp
        timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        temp_bag_name = f"bag_{timestamp}"
        self.current_output_folder = output_folder # Store for later renaming
        self.current_temp_bag_name = temp_bag_name # Store for later renaming

        # Construct the ros2 bag record command
        command = ['ros2', 'bag', 'record', '-o', os.path.join(output_folder, temp_bag_name)] + topics_to_record

        try:
            # Start the subprocess
            self.ros_process = QProcess(self)
            self.ros_process.readyReadStandardOutput.connect(self.handle_stdout)
            self.ros_process.readyReadStandardError.connect(self.handle_stderr)
            self.ros_process.started.connect(self.process_started)
            self.ros_process.finished.connect(self.process_finished)

            self.info_label.setText(f"Starting recording: {' '.join(command)}")
            self.ros_process.start('bash', ['-c', ' '.join(command)])

            self.record_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            CustomMessageBox.information(self, 'Recording Started', f'Recording to temporary name: {os.path.join(output_folder, temp_bag_name)}')

        except Exception as e:
            CustomMessageBox.critical(self, 'Error', f'Failed to start recording: {e}\n'
                                                 'Ensure ROS2 environment is sourced and "ros2" command is available in your PATH.')
            self.record_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            self.info_label.setText("Configure paths and click 'Record' to start recording ROS2 bags.")
            # Clear stored temporary names if recording failed to start
            self.current_output_folder = None
            self.current_temp_bag_name = None

    def stop_recording(self):
        """Handles the logic for stopping ROS2 bag recording."""
        if self.ros_process and self.ros_process.state() == QProcess.Running:
            self.ros_process.terminate()
            if not self.ros_process.waitForFinished(5000):
                self.ros_process.kill()

            self.info_label.setText("Recording stopped.")
            CustomMessageBox.information(self, 'Recording Stopped', 'ROS2 bag recording has been stopped.')
        else:
            self.info_label.setText("No active recording to stop.")
            CustomMessageBox.information(self, 'No Active Recording', 'There is no active ROS2 bag recording process.')

        self.record_button.setEnabled(True)
        self.stop_button.setEnabled(False)

        self.record_button.setEnabled(True)
        self.stop_button.setEnabled(False)

    def handle_stdout(self):
        """Reads and prints standard output from the ROS2 process."""
        data = self.ros_process.readAllStandardOutput().data().decode().strip()
        if data:
            print(f"STDOUT: {data}")
            # You can update a log area in your UI if needed

    def handle_stderr(self):
        """Reads and prints standard error from the ROS2 process."""
        data = self.ros_process.readAllStandardError().data().decode().strip()
        if data:
            print(f"STDERR: {data}")
            # You can update a log area in your UI if needed

    def process_started(self):
        """Called when the QProcess starts."""
        print("ROS2 bag record process started.")
        self.info_label.setText("Recording in progress...")

    def process_finished(self, exitCode, exitStatus):
            """Called when the QProcess finishes."""
            print(f"ROS2 bag record process finished with exit code {exitCode} and status {exitStatus}.")

            if exitStatus == QProcess.NormalExit and exitCode == 0:
                self.info_label.setText("Recording completed successfully.")
                if self.current_output_folder and self.current_temp_bag_name:
                    temp_bag_path = os.path.join(self.current_output_folder, self.current_temp_bag_name)

                    # Extract the timestamp from the temporary name
                    # Assumes format "bag_YYYY-MM-DD-HH-MM-SS"
                    timestamp_part = ""
                    if self.current_temp_bag_name.startswith("bag_") and len(self.current_temp_bag_name) > 4:
                        timestamp_part = self.current_temp_bag_name[4:]

                    # new_bag_name, ok = QInputDialog.getText(self, 'Rename Bag',
                    #                                         f'Recording saved as "{self.current_temp_bag_name}".\n'
                    #                                         'Enter a new name for the bag file (leave empty to keep current name):')
                    # Create QInputDialog and apply stylesheet for bigger text
                    input_dialog = QInputDialog(self)
                    input_dialog.setWindowTitle('Rename Bag')
                    input_dialog.setLabelText(f'Recording saved as "{self.current_temp_bag_name}".\n'
                                            'Enter a new name for the bag file (leave empty to keep current name):')
                    input_dialog.setStyleSheet("font-size: 20px;") # Increase font size for a larger dialog
                    ok = input_dialog.exec_()
                    new_bag_name = input_dialog.textValue() # This line retrieves the text

                    if ok and new_bag_name:
                        final_bag_name = f"{new_bag_name}_{timestamp_part}" if timestamp_part else new_bag_name
                        final_bag_path = os.path.join(self.current_output_folder, final_bag_name)
                        try:
                            os.rename(temp_bag_path, final_bag_path)
                            CustomMessageBox.information(self, 'Bag Renamed', f'Bag renamed to: {final_bag_name}')
                        except OSError as e:
                            CustomMessageBox.critical(self, 'Rename Error', f'Could not rename bag: {e}\n'
                                                                        f'Bag remains as: {self.current_temp_bag_name}')
                    else:
                        CustomMessageBox.information(self, 'Rename Skipped', f'Bag remains with temporary name: {self.current_temp_bag_name}')
                else:
                    CustomMessageBox.warning(self, 'Warning', 'Could not find temporary bag name or output folder for renaming.')

            else:
                self.info_label.setText(f"Recording finished with errors (Code: {exitCode}).")
                CustomMessageBox.warning(self, "Recording Finished", f"Recording process ended. Exit Code: {exitCode}, Status: {exitStatus}")

            self.record_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            self.ros_process = None # Clear the process reference
            self.current_output_folder = None # Clear stored data
            self.current_temp_bag_name = None # Clear stored data

    def closeEvent(self, event):
        """Handles the window close event, ensuring the ROS2 process is terminated."""
        if self.ros_process and self.ros_process.state() == QProcess.Running:
            reply = CustomMessageBox.question(self, 'Confirm Exit',
                                         "A recording is in progress. Do you want to stop it and exit?",
                                         CustomMessageBox.Yes | CustomMessageBox.No, CustomMessageBox.No)
            if reply == CustomMessageBox.Yes:
                self.stop_recording() # Attempt to stop gracefully
                if self.ros_process and self.ros_process.state() == QProcess.Running:
                    self.ros_process.kill() # Force kill if still running
                event.accept()
            else:
                event.ignore()
        else:
            event.accept()

def main(args=None):
    """Main function to run the ROS2 Bag Recorder GUI."""
    app = QApplication(sys.argv)
    ex = Ros2BagRecorderApp()
    ex.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()