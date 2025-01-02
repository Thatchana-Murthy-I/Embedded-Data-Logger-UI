import sys
import math
import random
import rasterio
import openpyxl
import serial
import os
import pandas as pd
import matplotlib.pyplot as plt
from PyQt5.QtCore import Qt, QTimer
from serial.tools import list_ports
from rasterio.transform import rowcol
from datetime import datetime, timedelta
from lac_analyzer_4 import Ui_MainWindow
from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtGui import QPixmap, QImage, QPainter, QColor
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtWidgets import (QApplication, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QMessageBox,
                             QVBoxLayout, QPushButton, QHBoxLayout, QWidget, QLineEdit, QLabel, QFileDialog)


class SerialReader(QtCore.QThread):
    sentence = QtCore.pyqtSignal(str)
    elsd_data = QtCore.pyqtSignal(str, str, str, str)
    time_data = QtCore.pyqtSignal(str, str, str)
    coordinate_data = QtCore.pyqtSignal(float, float, float)
    dop_data = QtCore.pyqtSignal(float, float, float)
    nmea_lineedits = QtCore.pyqtSignal(float, float, float, float, float, float, str, str, str, str)
    gpgsv_signal = QtCore.pyqtSignal(list, list)
    gigsv_signal = QtCore.pyqtSignal(list, list)
    glgsv_signal = QtCore.pyqtSignal(list, list)

    def __init__(self, port, baudrate):
        super(SerialReader, self).__init__()
        self.port = port
        self.baudrate = baudrate
        self.should_run = True
        self.ser = None

        self.data = None
        self.gsv_sentences = []
        self.glgsv_sentences = []
        self.gps_sentences = []

        self.PRN_list_gps = []
        self.SNR_list_gps = []
        self.PRN_list_navic = []
        self.SNR_list_navic = []
        self.PRN_list_glonass = []
        self.SNR_list_glonass = []

    def stop(self):
        self.should_run = False

    def run(self):
        try:
            self.ser = serial.Serial(self.port, int(self.baudrate), timeout=1)
            print(f"Opened serial port: {self.port}")

            while self.should_run:
                try:

                    line = self.ser.readline()

                    # Decode the line safely, using 'replace' to avoid crashes on bad bytes
                    nmea_sentence = line.decode('utf-8', errors='replace').strip()

                    self.data = nmea_sentence
                    split_data = self.data.split(',')
                    if len(split_data) > 2 and split_data[2] == 'A':
                        print(f"Valid NMEA data: {split_data}")
                        self.sentence.emit(nmea_sentence)
                        self.parse_nmea(nmea_sentence)

                    else:
                        print(split_data)
                    # Check if the sentence has sufficient elements before accessing split_data[2]
                    # if split_data[2] == "A":
                    #

                    # else:
                    #     print("No fix or incomplete data in NMEA sentence.")

                except UnicodeDecodeError as e:
                    print(f"Decoding error: {e}")

        except Exception as e:
            print(f"Error reading from serial port: {e}")

        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print(f"Closed serial port: {self.port}")

    def parse_nmea(self, nmea_sentence):
        try:
            if not nmea_sentence:
                print("Invalid or empty NMEA sentence.")
                return

            self.data = nmea_sentence
            split_data = self.data.split(',')

            if split_data[0] == "$ELSD":
                self.handle_elsd_data(split_data)

            elif split_data[0] == "$GNRMC":
                self.handle_gnrmc_data(split_data)

            elif split_data[0] == "$GNGGA":
                self.handle_gngga_data(split_data)

            elif split_data[0] == "$GNGSA":
                self.handle_gngsa_data(split_data)

            elif split_data[0] == "$GPGSV":
                self.handle_gpgsv(split_data)

            elif split_data[0] == "$GIGSV":
                self.handle_gigsv(split_data)

            elif split_data[0] == "$GLGSV":
                self.handle_glgsv(split_data)

        except Exception as e:
            print(f"Error parsing NMEA sentence: {e}")

    def handle_gpgsv(self, split_data):
        try:
            if len(split_data) < 4 or not split_data[1].isdigit() or not split_data[2].isdigit():
                print("Incomplete or invalid GPGSV data.")
                return

            sentence_count = int(split_data[1])
            current_sentence = int(split_data[2])

            self.gps_sentences.append(split_data)
            print(f"GPGSV Sentence: {split_data}")

            if current_sentence == sentence_count:
                self.handle_gpgsv_sentences(self.gps_sentences)
                self.gps_sentences.clear()

        except Exception as e:
            print(f"Exception in handle_gpgsv: {e}")

    def handle_gigsv(self, split_data):
        try:
            if len(split_data) > 1 and split_data[1].isdigit() and split_data[2].isdigit():
                sentence_count = int(split_data[1])
                current_sentence = int(split_data[2])

                self.gsv_sentences.append(split_data)
                print(f"GIGSV Sentence: {split_data}")

                if current_sentence == sentence_count:
                    self.handle_gigsv_sentences(self.gsv_sentences)
                    self.gsv_sentences.clear()

            else:
                print("Invalid GSV sentence format or count is not numeric.")

        except Exception as e:
            print(f"Exception in handle_gpgsv: {e}")

    def handle_glgsv(self, split_data):
        try:
            if len(split_data) > 1 and split_data[1].isdigit() and split_data[2].isdigit():
                sentence_count = int(split_data[1])
                current_sentence = int(split_data[2])

                self.glgsv_sentences.append(split_data)
                print(f"GLGSV Sentence: {split_data}")

                if current_sentence == sentence_count:
                    self.handle_glgsv_sentences(self.glgsv_sentences)
                    self.glgsv_sentences.clear()

            else:
                print("Invalid GSV sentence format or count is not numeric.")

        except Exception as e:
            print(f"Exception in handle_gpgsv: {e}")

    def handle_gpgsv_sentences(self, gps_sentences):
        try:
            for gps in gps_sentences:
                for i in range(4, len(gps) - 1):
                    if i % 4 == 0 and gps[i]:  # PRN indices
                        self.PRN_list_gps.append(gps[i])
                    elif i % 4 == 3 and gps[i]:  # SNR indices
                        self.SNR_list_gps.append(gps[i])
                    elif i % 4 == 3:  # Handle missing SNR values
                        self.SNR_list_gps.append(0)

            print("PRN list (GPS):", self.PRN_list_gps)
            print("SNR list (GPS):", self.SNR_list_gps)
            self.gpgsv_signal.emit(self.PRN_list_gps, self.SNR_list_gps)
            print("_________________________________________\n")

            self.PRN_list_gps.clear()
            self.SNR_list_gps.clear()

        except Exception as e:
            print(f"Exception in handle_gpgsv_sentences: {e}")

    def handle_gigsv_sentences(self, gsv_sentences):
        try:
            for gsv in gsv_sentences:
                for i in range(4, len(gsv) - 1):
                    if i % 4 == 0 and gsv[i]:  # PRN indices
                        self.PRN_list_navic.append(gsv[i])
                    elif i % 4 == 3 and gsv[i]:  # SNR indices
                        self.SNR_list_navic.append(gsv[i])
                    elif i % 4 == 3:  # Handle missing SNR values
                        self.SNR_list_navic.append(0)

            print("PRN list (Navic):", self.PRN_list_navic)
            print("SNR list (Navic):", self.SNR_list_navic)
            self.gigsv_signal.emit(self.PRN_list_navic, self.SNR_list_navic)

            print("_________________________________________\n")

            # Clear PRN and SNR lists after processing
            self.PRN_list_navic.clear()
            self.SNR_list_navic.clear()

        except Exception as e:
            print(f"Exception in handle_gigsv_sentences: {e}")

    def handle_glgsv_sentences(self, glgsv_sentences):
        try:
            for glgsv in glgsv_sentences:
                for i in range(4, len(glgsv) - 1):
                    if i % 4 == 0 and glgsv[i]:
                        self.PRN_list_glonass.append(glgsv[i])
                    elif i % 4 == 3 and glgsv[i]:
                        self.SNR_list_glonass.append(glgsv[i])
                    elif i % 4 == 3:  # Handle missing SNR values
                        self.SNR_list_glonass.append(0)

            print("PRN list (Glonass):", self.PRN_list_glonass)
            print("SNR list (Glonass):", self.SNR_list_glonass)
            self.glgsv_signal.emit(self.PRN_list_glonass, self.SNR_list_glonass)
            print("_________________________________________\n")

            self.PRN_list_glonass.clear()
            self.SNR_list_glonass.clear()

        except Exception as e:
            print(f"Exception in handle_gpgsv_sentences: {e}")

    def handle_elsd_data(self, split_data):
        try:
            temperature = split_data[1] if len(split_data) > 1 else "no_data"
            hpa = split_data[2] if len(split_data) > 2 else "no_data"
            humi = split_data[3] if len(split_data) > 3 else "no_data"
            sensor_alti = split_data[4] if len(split_data) > 4 else "no_data"
            self.elsd_data.emit(sensor_alti, hpa, temperature, humi)
            print([sensor_alti, hpa, temperature, humi])

        except Exception as e:
            print(f"Error handling ELSD data: {e}")

    def handle_gnrmc_data(self, split_data):
        try:
            time_now = f"{split_data[1][:2]}:{split_data[1][2:4]}:{split_data[1][4:6]}" if len(
                split_data) > 1 else "no_data"
            current_date = f"{split_data[9][:2]}-{split_data[9][2:4]}-{split_data[9][4:6]}" if len(
                split_data) > 9 else "no_data"
            time_obj = datetime.strptime(time_now, '%H:%M:%S')
            time_delta = timedelta(hours=5, minutes=30, seconds=2)
            ist_time = (time_obj + time_delta).time().strftime('%H:%M:%S')
            self.time_data.emit(ist_time, current_date, time_now)
            print(ist_time, current_date, time_now)
        except Exception as e:
            print(f"Error handling GNRMC data: {e}")

    def handle_gngga_data(self, split_data):
        try:
            if len(split_data) < 10:
                print("Incomplete GNGGA data.")
                self.coordinate_data.emit(0.0, 0.0, 0.0)
                return

            lat = (float(split_data[2][:2]) + float(split_data[2][2:]) / 60) if split_data[2] else 0.0
            lon = (float(split_data[4][:3]) + float(split_data[4][3:]) / 60) if split_data[4] else 0.0
            alti = float(split_data[9]) if split_data[9] else 0.0
            self.coordinate_data.emit(lat, lon, alti)
            print([lat, lon, alti])

        except Exception as e:
            print(f"Error handling GNGGA data: {e}")
            self.coordinate_data.emit(0.0, 0.0, 0.0)

    def handle_gngsa_data(self, split_data):
        try:
            if len(split_data) < 18:
                print("Incomplete GNGSA data.")
                self.dop_data.emit(0.0, 0.0, 0.0)
                return

            pdop = float(split_data[15]) if split_data[15] else 0.0
            hdop = float(split_data[16]) if split_data[16] else 0.0
            vdop = float(split_data[17]) if split_data[17] else 0.0
            self.dop_data.emit(pdop, hdop, vdop)
            print([pdop, hdop, vdop])

        except Exception as e:
            print(f"Error handling GNGSA data: {e}")
            self.dop_data.emit(0.0, 0.0, 0.0)


class MainWindow(QtWidgets.QMainWindow, QtCore.QThread):

    def __init__(self):
        super(MainWindow, self).__init__()

        # lat = 13.02305367  # Example latitude
        # lon = 77.585776  # Example longitude

        self.csv_file = None
        self.timer = None
        self.sensor_alti = 0
        self.hpa = 0
        self.temperature = None
        self.humi = None
        self.ist_time = None
        self.current_date = None
        self.time_now = None
        self.lat = None
        self.lon = None
        self.alti = None
        self.pdop = None
        self.hdop = None
        self.vdop = None

        self.init_ui()
        self.alti_list = []
        self.sensor_alti_list = []

        self.PRN_list_navic = []
        self.SNR_list_navic = []
        self.PRN_list_gps = []
        self.SNR_list_gps = []
        self.PRN_list_glonass = []
        self.SNR_list_glonass = []

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.retranslateUi(self)
        self.is_receiver_connected = False
        self.serial_reader = None
        self.populate_Serial_ports()
        self.populate_baud_rates()

        # self.ui.map_widget = QtWidgets.QLabel(self.ui.centralwidget)
        # self.ui.map_widget.setGeometry(QtCore.QRect(710, 30, 431, 361))
        # self.ui.map_widget.setObjectName("map_widget")
        # self.viewer = TIFFViewer(self.ui.map_widget, lat, lon)

        self.ui.pan_bt.setCheckable(True)

        # Button Actions
        self.ui.connect_pushButton.clicked.connect(self.toggle_function)
        self.ui.refresh_pushButton.clicked.connect(self.populate_Serial_ports)
        # self.ui.zoom_out_bt.clicked.connect(self.viewer.zoom_out)
        # self.ui.zoom_in_bt.clicked.connect(self.viewer.zoom_in)
        self.ui.record_pushButton.clicked.connect(self.toggle_recording)
        #
        # self.ui.pan_bt.clicked.connect(self.viewer.toggle_pan)
        self.ui.download_Button.clicked.connect(self.save_to_text_file)

        # Modify the lineEdits to readonly and align to centre
        self.ui.lat_lineEdit.setReadOnly(True)
        self.ui.lat_lineEdit.setAlignment(Qt.AlignCenter)
        self.ui.lon_lineEdit.setReadOnly(True)
        self.ui.lon_lineEdit.setAlignment(Qt.AlignCenter)
        self.ui.alt_lineEdit.setReadOnly(True)
        self.ui.alt_lineEdit.setAlignment(Qt.AlignCenter)
        self.ui.alt2_lineEdit.setReadOnly(True)
        self.ui.alt2_lineEdit.setAlignment(Qt.AlignCenter)
        self.ui.temp_lineEdit.setReadOnly(True)
        self.ui.temp_lineEdit.setAlignment(Qt.AlignCenter)
        self.ui.humi_lineEdit.setReadOnly(True)
        self.ui.humi_lineEdit.setAlignment(Qt.AlignCenter)
        self.ui.pressure_lineEdit.setReadOnly(True)
        self.ui.pressure_lineEdit.setAlignment(Qt.AlignCenter)
        self.ui.utc_lineEdit.setReadOnly(True)
        self.ui.utc_lineEdit.setAlignment(Qt.AlignCenter)
        self.ui.ist_lineEdit.setReadOnly(True)
        self.ui.ist_lineEdit.setAlignment(Qt.AlignCenter)
        self.ui.date_lineEdit.setReadOnly(True)
        self.ui.date_lineEdit.setAlignment(Qt.AlignCenter)
        self.ui.pdop_lineEdit.setReadOnly(True)
        self.ui.pdop_lineEdit.setAlignment(Qt.AlignCenter)
        self.ui.pdop_lineEdit.setAlignment(Qt.AlignCenter)
        self.ui.hdop_lineEdit.setReadOnly(True)
        self.ui.hdop_lineEdit.setAlignment(Qt.AlignCenter)
        self.ui.vdop_lineEdit.setReadOnly(True)
        self.ui.vdop_lineEdit.setAlignment(Qt.AlignCenter)

        # Modify the nmea_textedits to readonly
        self.ui.nmea_plainTextEdit.setReadOnly(True)
        self.ui.nmea_plainTextEdit.moveCursor(QtGui.QTextCursor.End)

        # Set initial state
        self.satellite_graph = Satellite_graph(self.ui.plot_widget)
        # self.my_instance = SNR_view(self.satellite_graph)
        # # Connect the signal to the slot for updating the graph
        # self.my_instance.update_graph_signal.connect(self.satellite_graph.update_graph)

        # Start the thread
        # self.my_instance.start()
        self.is_running = False
        self.worker = None
        self.is_recording = False  # Tracks the recording state

    def init_ui(self):
        # Set up the UI using the auto-generated class
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

    def toggle_function(self):
        if self.is_running:

            self.stop_function()
        else:
            # Start the function
            self.is_running = True

            self.start_serial_reader()

    def save_to_text_file(self):
        # Retrieve the text from nmea_plainTextEdit
        text = self.ui.nmea_plainTextEdit.toPlainText()

        # Open a file dialog to select the save location with a default file name
        options = QFileDialog.Options()
        default_filename = "NMEA_Data.txt"  # Default filename that user can change
        file_path, _ = QFileDialog.getSaveFileName(self, "Save NMEA Data", default_filename,
                                                   "Text Files (*.txt);;All Files (*)", options=options)

        if file_path:
            # Write the content to a .txt file
            with open(file_path, "w") as file:
                file.write(text)

    def stop_function(self):
        # Set flag to indicate function is stopped
        self.is_running = False
        self.serial_reader.stop()
        self.ui.refresh_pushButton.setEnabled(True)
        self.ui.connect_pushButton.setStyleSheet("background-image: url(:/icon/connect.jpg);\n"
                                                 "image: url(:/icon/connect.jpg);")
        self.is_receiver_connected = False

    def populate_Serial_ports(self):
        self.ui.receiver_comboBox.clear()
        ports = list_ports.comports()
        for port in ports:
            print(ports)
            self.ui.receiver_comboBox.addItem(port.device)

    def populate_baud_rates(self):
        baud_rates = ['--SELECT--', '115200', '9600']
        for baud_rate in baud_rates:
            self.ui.baudrate_comboBox.addItem(baud_rate)

    def start_serial_reader(self):
        try:
            self.ui.refresh_pushButton.setEnabled(False)
            self.ui.connect_pushButton.setStyleSheet("background-image: url(:/icon/disconnect.jpg);\n"
                                                     "image: url(:/icon/disconnect.jpg);"
                                                     "colour : red;")
            try:
                if not self.is_receiver_connected:
                    port = self.ui.receiver_comboBox.currentText()
                    baudrate = self.ui.baudrate_comboBox.currentText()

                    if self.serial_reader and self.serial_reader.isRunning():
                        print("Serial reader is already running...")
                        return

                    self.serial_reader = SerialReader(port, baudrate)
                    # Ensure parse_gga exists in the MainWindow class
                    # if hasattr(self, 'parse_gga'):
                    #     self.serial_reader.data_received.connect(self.parse_gga)
                    # else:
                    #     print("Error: 'parse_gga' method is not defined in MainWindow.")
                    #     return

                    self.serial_reader.start()
                    self.serial_reader.sentence.connect(self.nmea_sentence)
                    self.serial_reader.elsd_data.connect(self.handle_elsd_data)
                    self.serial_reader.time_data.connect(self.handle_time_data)
                    self.serial_reader.coordinate_data.connect(self.handle_coordinate_data)
                    self.serial_reader.dop_data.connect(self.handle_dop_data)
                    self.serial_reader.gpgsv_signal.connect(self.handle_gpgsv_sentences)
                    self.serial_reader.gigsv_signal.connect(self.handle_gigsv_sentences)
                    self.serial_reader.glgsv_signal.connect(self.handle_glgsv_sentences)

                    # if sentence.startswith("$G"):
                    #     self.ui.nmea_plainTextEdit.appendPlainText(sentence)
                    print("Module Port:", port, "Baudrate:", baudrate)

                else:
                    if self.serial_reader and self.serial_reader.isRunning():
                        self.serial_reader.stop()
                        self.serial_reader.wait()
                        print("Serial reader stopped.")
                    else:
                        print("No active serial reader.")

            except Exception as e:
                print(f"Error starting SerialReader: {e}")
        except Exception as e:
            print(f"Error starting SerialReader: {e}")

    def nmea_sentence(self, sentence):
        if sentence.startswith("$G"):
            self.ui.nmea_plainTextEdit.appendPlainText(sentence)

    def handle_glgsv_sentences(self, prn_list, snr_list):
        self.satellite_graph.update_graph("GLONASS", prn_list, snr_list)

    def handle_gigsv_sentences(self, prn_list, snr_list):
        self.satellite_graph.update_graph("NavIC", prn_list, snr_list)

    def handle_gpgsv_sentences(self, prn_list, snr_list):
        self.satellite_graph.update_graph("GPS", prn_list, snr_list)

    def handle_elsd_data(self, sensor_alti, hpa, temperature, humi):
        self.sensor_alti = sensor_alti
        self.hpa = hpa
        self.temperature = temperature
        self.humi = humi

        self.sensor_alti_list.append(sensor_alti)

        try:
            if sensor_alti is None or hpa is None or temperature is None or humi is None:
                print(f"BMP not working")
            else:
                print(f"Sensor Data - Altitude: {sensor_alti}, HPA: {hpa}, Temperature: {temperature}, Humidity: {humi}")

            self.ui.temp_lineEdit.clear()
            self.ui.pressure_lineEdit.clear()
            self.ui.humi_lineEdit.clear()
            self.ui.alt2_lineEdit.clear()

            self.ui.temp_lineEdit.setText(str(temperature))
            self.ui.pressure_lineEdit.setText(str(hpa))
            self.ui.humi_lineEdit.setText(str(humi))
            self.ui.alt2_lineEdit.setText(str(sensor_alti))

        except Exception as e:
            print(f"Exception in handle_elsd_data {e}")

    def handle_time_data(self, ist_time, current_date, time_now):

        self.ist_time = ist_time
        self.current_date = current_date
        self.time_now = time_now

        try:
            if ist_time is None or current_date is None or time_now is None:
                print(f"No Fix")

            else:
                print(f"Parse Time : {ist_time, current_date, time_now}")

                self.ui.utc_lineEdit.clear()
                self.ui.ist_lineEdit.clear()
                self.ui.date_lineEdit.clear()

                self.ui.utc_lineEdit.setText(f"{time_now} hrs")
                self.ui.ist_lineEdit.setText(f"{ist_time} hrs")
                self.ui.date_lineEdit.setText(current_date)

        except Exception as e:
            print(f"Exception in handle_time_data {e}")

    def handle_coordinate_data(self, lat, lon, alti):

        self.lat = lat
        self.lon = lon
        self.alti = alti
        self.alti_list.append(alti)
        try:

            if lat is None or lon is None or alti is None:
                print(f"No Fix")
            else:
                print(f"Coordinates: Lat={lat}, Lon={lon}, Altitude={alti}")

                self.ui.lat_lineEdit.clear()
                self.ui.lon_lineEdit.clear()
                self.ui.alt_lineEdit.clear()

                self.ui.lat_lineEdit.setText(f"{lat:.6f}")
                self.ui.lon_lineEdit.setText(f"{lon:.6f}")
                self.ui.alt_lineEdit.setText(f"{alti:.2f} m")
        except Exception as e:
            print(f"Exception in handle_coordinate_data: {e}")

    def handle_dop_data(self, pdop, hdop, vdop):

        self.pdop = pdop
        self.hdop = hdop
        self.vdop = vdop
        try:
            if pdop is None or hdop is None or vdop is None:
                print(f"No Fix")
            else:
                print(f"DOP: PDOP={pdop}, HDOP={hdop}, VDOP={vdop}")

                self.ui.pdop_lineEdit.clear()
                self.ui.hdop_lineEdit.clear()
                self.ui.vdop_lineEdit.clear()

                # Convert float to string
                self.ui.pdop_lineEdit.setText(f"{pdop:.2f}")
                self.ui.hdop_lineEdit.setText(f"{hdop:.2f}")
                self.ui.vdop_lineEdit.setText(f"{vdop:.2f}")

        except Exception as e:
            print(f"Exception in handle_dop_data: {e}")

    def toggle_recording(self):
        """Toggle recording on and off."""
        try:
            if self.is_recording:
                # Stop the recording
                self.is_recording = False
                print("Recording stopped.")
            else:
                # Start the recording
                self.is_recording = True
                print("Recording started.")

                # Open the file dialog to save the Excel file
                options = QFileDialog.Options()
                default_filename = "NMEA_Data.xlsx"  # Default filename
                file_path, _ = QFileDialog.getSaveFileName(self, "Save NMEA Data", default_filename,
                                                           "Excel Files (*.xlsx);;All Files (*)", options=options)
                if file_path:
                    # Set the selected file path
                    self.csv_file = file_path  # Set the file path
                    self.ui.folder_lineEdit.setText(str(file_path))

                    # Check if the file exists
                    if not os.path.exists(self.csv_file):
                        # If the file does not exist, create a new Excel file with headers
                        with pd.ExcelWriter(self.csv_file, engine="openpyxl", mode="w") as writer:
                            df = pd.DataFrame(columns=[
                                "Lat", "Long", "Time (UTC)", "Time (IST)", "Time Diff",
                                "Temp", "Air Pr", "Humid", "Alti (NDNU)",
                                "Alti (Sensor)", "Alti Vari (NMEA)",
                                "Alti Vari (Sensor)", "Alti Vari (NMEA-Sensor)"
                            ])
                            df.to_excel(writer, sheet_name="Master Sheet", index=False)
                        print(f"Recording initialized and file saved at: {self.csv_file}")

                    # Start the timer to append data every 5 seconds
                    self.timer = QtCore.QTimer(self)
                    self.timer.timeout.connect(self.append_coordinate_data)
                    self.timer.start(5000)  # Timeout interval in milliseconds (5000ms = 5 seconds)
        except Exception as e:
            print(f"Error in Toggle recording: {e}")

    def append_coordinate_data(self):
        """Append the coordinate data to the Excel file every 5 seconds."""
        try:
            # Convert to datetime objects
            # time_now = datetime.strptime(self.time_now, "%Y-%m-%d %H:%M:%S")
            # ist_time = datetime.strptime(self.ist_time, "%Y-%m-%d %H:%M:%S")
            #
            # time_difference = ist_time - time_now

            if self.lat is not None and self.lon is not None and self.alti is not None and len(self.sensor_alti_list) >= 2:
                with pd.ExcelWriter(self.csv_file, engine="openpyxl", mode="a", if_sheet_exists="overlay") as writer:
                    # Prepare data to write
                    data = [[self.lat, self.lon, self.time_now, self.ist_time, "time_difference", self.temperature, self.hpa, self.humi,
                             self.alti, self.sensor_alti, "Alti Vari (NMEA)", "Alti (Sensor)",
                             "Alti Vari (NMEA-Sensor)"]]
                    df = pd.DataFrame(data, columns=[
                        "Lat", "Long", "Time (UTC)", "Time (IST)", "Time Diff",
                        "Temp", "Air Pr", "Humid", "Alti (NDNU)",
                        "Alti (Sensor)", "Alti Vari (NMEA)",
                        "Alti Vari (Sensor)", "Alti Vari (NMEA-Sensor)"
                    ])
                    # Write the data to the Excel sheet without overwriting the header
                    df.to_excel(writer, sheet_name="Master Sheet", index=False, header=False,
                                startrow=writer.sheets["Master Sheet"].max_row)
                print(f"Coordinate data written to {self.csv_file}")
            else:
                print("No coordinate data available.")
        except Exception as e:
            print(f"Error in append_coordinate_data: {e}")

    # def toggle_recording(self):
    #     """Toggle the recording state and update the button text."""
    #     try:
    #         if self.is_recording:
    #             # Save the file when stopping the recording
    #             try:
    #                 print(f"Recording stopped. Data saved to: {self.csv_file}")
    #             except Exception as e:
    #                 print(f"Error saving file: {e}")
    #
    #         self.is_recording = not self.is_recording
    #         self.ui.record_pushButton.setText("Stop Recording" if self.is_recording else "Start Recording")
    #
    #         if self.is_recording:
    #             # Open file dialog to select the save location
    #             options = QFileDialog.Options()
    #             file_path, _ = QFileDialog.getSaveFileName(
    #                 self,
    #                 "Save Excel File",
    #                 "Data_Record.xlsx",  # Default filename
    #                 "Excel Files (*.xlsx);;All Files (*)",
    #                 options=options
    #             )
    #
    #             if file_path:  # Proceed only if the user selects a file
    #                 if not file_path.endswith(".xlsx"):
    #                     file_path += ".xlsx"  # Ensure the correct file extension
    #
    #                 self.csv_file = file_path  # Set the selected file path
    #                 try:
    #                     # Initialize the Excel file at the chosen location
    #                     with pd.ExcelWriter(self.csv_file, engine="openpyxl", mode="w") as writer:
    #                         pd.DataFrame(columns=[
    #                             "Lat", "Long", "Time (UTC)", "Time (IST)", "Time Diff",
    #                             "Temp", "Air Pr", "Humid", "Alti (NDNU)",
    #                             "Alti (Sensor)", "Alti Vari (NMEA)",
    #                             "Alti Vari (Sensor)", "Alti Vari (NMEA-Sensor)"
    #                         ]).to_excel(writer, sheet_name="Master Sheet", index=False)
    #                     print(f"Recording initialized and file saved at: {self.csv_file}")
    #                 except Exception as e:
    #                     print(f"Error initializing file: {e}")
    #             else:
    #                 # If no file is selected, disable recording and revert the button text
    #                 self.is_recording = False
    #                 self.ui.record_pushButton.setText("Start Recording")
    #                 print("Recording cancelled. No file selected.")
    #     except Exception as e:
    #         print(f"Error in toggle_recording: {e}")
    #
    # def update_data(self):
    #     """Update data values and handle recording."""
    #     try:
    #         if self.is_recording:
    #             # Ensure attributes are initialized
    #             self.sensor_alti = self.sensor_alti or 0
    #             self.hpa = self.hpa or 0
    #             self.temperature = self.temperature or 0
    #             self.humi = self.humi or 0
    #             self.ist_time = self.ist_time or QDateTime.currentDateTime().toString("HH:mm:ss")
    #             self.current_date = self.current_date or QDateTime.currentDateTime().toString("yyyy-MM-dd")
    #             self.time_now = self.time_now or QDateTime.currentDateTime().toString("HH:mm:ss")
    #             self.lat = self.lat or 0
    #             self.lon = self.lon or 0
    #             self.alti = self.alti or 0
    #             self.pdop = self.pdop or 0
    #             self.hdop = self.hdop or 0
    #             self.vdop = self.vdop or 0
    #
    #             # Append the latest sensor values to the data list
    #             self.data_list.append({
    #                 "Sensor Altitude": self.sensor_alti,
    #                 "Pressure (hPa)": self.hpa,
    #                 "Temperature (°C)": self.temperature,
    #                 "Humidity (%)": self.humi,
    #                 "IST Time": self.ist_time,
    #                 "Current Date": self.current_date,
    #                 "Current Time": self.time_now,
    #                 "Latitude": self.lat,
    #                 "Longitude": self.lon,
    #                 "Altitude (NMEA)": self.alti,
    #                 "PDOP": self.pdop,
    #                 "HDOP": self.hdop,
    #                 "VDOP": self.vdop
    #             })
    #
    #             # If at least two IST times are available, calculate differences
    #             if len(self.time_ist_list) >= 2:
    #                 time_diff = datetime.strptime(self.time_ist_list[-1], "%H:%M:%S") - datetime.strptime(
    #                     self.time_ist_list[-2], "%H:%M:%S")
    #                 self.data_list[-1].update({"Time Difference": time_diff})
    #
    #             # Write the latest entry to the Excel file
    #             with pd.ExcelWriter(self.csv_file, engine="openpyxl", mode="a",
    #                                 if_sheet_exists="overlay") as writer:
    #                 pd.DataFrame([self.data_list[-1]]).to_excel(
    #                     writer,
    #                     sheet_name="Master Sheet",
    #                     index=False,
    #                     header=False,
    #                     startrow=writer.sheets["Master Sheet"].max_row,
    #                 )
    #     except Exception as e:
    #         print(f"Error while updating data: {e}")


class Satellite_graph(QtWidgets.QFrame, QtCore.QThread):
    def __init__(self, plot_widget):
        super().__init__()
        self.plot_widget = plot_widget  # Store plot widget reference
        self.init_ui()
        # Adjust bottom margin
        bottom_margin = 0.2

        # Initialize Matplotlib figures and axes for NavIC, GPS, and GLONASS
        self.figure_navic, self.ax_navic = plt.subplots()
        self.figure_navic.subplots_adjust(bottom=bottom_margin)
        self.ax_navic.set_title("NavIC")
        self.ax_navic.set_xlabel("PRN")
        self.ax_navic.set_ylabel("SNR")

        self.figure_gps, self.ax_gps = plt.subplots()
        self.figure_gps.subplots_adjust(bottom=bottom_margin)
        self.ax_gps.set_title("GPS")
        self.ax_gps.set_xlabel("PRN")
        self.ax_gps.set_ylabel("SNR")

        self.figure_glonass, self.ax_glonass = plt.subplots()
        self.figure_glonass.subplots_adjust(bottom=bottom_margin)
        self.ax_glonass.set_title("GLONASS")
        self.ax_glonass.set_xlabel("PRN")
        self.ax_glonass.set_ylabel("SNR")

        # Create canvases for each figure
        self.canvas_navic = FigureCanvas(self.figure_navic)
        self.canvas_gps = FigureCanvas(self.figure_gps)
        self.canvas_glonass = FigureCanvas(self.figure_glonass)

        # Add borders to the canvases using a QFrame
        self.navic_frame = QtWidgets.QFrame()
        self.navic_frame.setFrameShape(QtWidgets.QFrame.Box)
        self.navic_frame.setLineWidth(1)
        self.navic_layout = QtWidgets.QVBoxLayout(self.navic_frame)
        self.navic_layout.addWidget(self.canvas_navic)

        self.gps_frame = QtWidgets.QFrame()
        self.gps_frame.setFrameShape(QtWidgets.QFrame.Box)
        self.gps_frame.setLineWidth(1)
        self.gps_layout = QtWidgets.QVBoxLayout(self.gps_frame)
        self.gps_layout.addWidget(self.canvas_gps)

        self.glonass_frame = QtWidgets.QFrame()
        self.glonass_frame.setFrameShape(QtWidgets.QFrame.Box)
        self.glonass_frame.setLineWidth(1)
        self.glonass_layout = QtWidgets.QVBoxLayout(self.glonass_frame)
        self.glonass_layout.addWidget(self.canvas_glonass)

        # Create a vertical layout for plot_widget to hold the framed canvases
        vertical_layout = QtWidgets.QVBoxLayout(self.plot_widget)
        vertical_layout.addWidget(self.navic_frame)
        vertical_layout.addWidget(self.gps_frame)
        vertical_layout.addWidget(self.glonass_frame)

        # # Create a vertical layout for plot_widget to hold the canvases
        # vertical_layout = QtWidgets.QVBoxLayout(self.plot_widget)
        # vertical_layout.addWidget(self.canvas_navic)
        # vertical_layout.addWidget(self.canvas_gps)
        # vertical_layout.addWidget(self.canvas_glonass)

        # Lists for storing data for the graphs
        self.PRN_list_navic = []
        self.SNR_list_navic = []

        self.PRN_list_gps = []
        self.SNR_list_gps = []

        self.PRN_list_glonass = []
        self.SNR_list_glonass = []

    def init_ui(self):
        pass

    def update_graph(self, satellite_type, PRN, SNR):
        if satellite_type == "NavIC":
            ax = self.ax_navic
            canvas = self.canvas_navic
            PRN_list = self.PRN_list_navic
            SNR_list = self.SNR_list_navic
            color = "#obfc03"  # Green bar

        elif satellite_type == "GPS":
            ax = self.ax_gps
            canvas = self.canvas_gps
            PRN_list = self.PRN_list_gps
            SNR_list = self.SNR_list_gps
            color = "#1f77b4"  # Blue bar

        elif satellite_type == "GLONASS":
            ax = self.ax_glonass
            canvas = self.canvas_glonass
            PRN_list = self.PRN_list_glonass
            SNR_list = self.SNR_list_glonass
            color = "#ff7f0e"  # Orange bar
        else:
            return

        PRN_list.clear()
        SNR_list.clear()
        PRN_list.extend(PRN)
        SNR_list.extend(SNR)

        self.plotgraph(ax, canvas, PRN_list, SNR_list, satellite_type, color)

    @staticmethod
    def plotgraph(ax, canvas, prn_list, snr_list, satellite_type, color):
        ax.clear()

        # Convert snr_list to integers if it contains string values
        snr_list = list(map(int, snr_list))

        # Plotting the bar graph
        ax.bar(prn_list, snr_list, color=color)

        # Set labels and title
        ax.set_xlabel("PRN")
        ax.set_ylabel("SNR")
        ax.set_title(satellite_type)

        # Set dynamic y-axis limits with padding for better proportion
        y_min = min(snr_list) - 2.5
        y_max = max(snr_list) + 2.5
        ax.set_ylim(y_min, y_max)

        # Redraw the canvas to reflect changes
        canvas.draw_idle()

# Example Usage in Main Code
# satellite_graph = SatelliteGraph(plot_widget)
# satellite_graph.update_graph("NavIC", [1, 2, 3], [30, 40, 50])


class TIFFViewer(QWidget):
    def __init__(self, parent, lat, lon):
        super().__init__(parent)

        # Initialize the zoom factor
        self.ripple_radius = None
        self.ripple_opacity = None
        self.ripple_timer = None
        self.pixmap_item = None
        self.zoom_factor = 1.0
        self.lat = lat
        self.lon = lon

        # Create the layout
        layout = QVBoxLayout(self)

        # Create the QGraphicsView and QGraphicsScene
        self.graphics_view = QGraphicsView(self)
        self.graphics_scene = QGraphicsScene(self)

        # Set the QGraphicsScene on the QGraphicsView
        self.graphics_view.setScene(self.graphics_scene)
        self.graphics_view.setRenderHint(QPainter.Antialiasing)

        # Load and display TIFF image with a marker
        self.display_tiff_image('Gangagar.tif')

        # Add zoom in/out and center buttons
        button_layout = QHBoxLayout()
        zoom_in_button = QPushButton("Zoom In")
        zoom_in_button.clicked.connect(self.zoom_in)

        zoom_out_button = QPushButton("Zoom Out")
        zoom_out_button.clicked.connect(self.zoom_out)

        center_button = QPushButton("Center Map")
        center_button.clicked.connect(self.center_map)

        # Add a pan toggle button in the layout
        pan_toggle_button = QPushButton("Toggle Pan")
        pan_toggle_button.setCheckable(True)  # Make the button toggleable
        pan_toggle_button.toggled.connect(self.toggle_pan)

        current_location_button = QPushButton("Current Location")
        current_location_button.clicked.connect(self.center_on_current_location)

        button_layout.addWidget(pan_toggle_button)
        button_layout.addWidget(zoom_in_button)
        button_layout.addWidget(zoom_out_button)
        button_layout.addWidget(center_button)
        button_layout.addWidget(current_location_button)

        # Create input fields for latitude and longitude
        self.lat_input = QLineEdit(str(self.lat))
        self.lon_input = QLineEdit(str(self.lon))
        lat_label = QLabel("Latitude:")
        lon_label = QLabel("Longitude:")

        input_layout = QHBoxLayout()
        input_layout.addWidget(lat_label)
        input_layout.addWidget(self.lat_input)
        input_layout.addWidget(lon_label)
        input_layout.addWidget(self.lon_input)

        # Add the QGraphicsView, input fields, and buttons to the layout
        layout.addWidget(self.graphics_view)
        layout.addLayout(input_layout)
        layout.addLayout(button_layout)

        self.setLayout(layout)  # Set the layout for this widget

    def toggle_pan(self, enable):
        if enable:
            self.graphics_view.setDragMode(QGraphicsView.ScrollHandDrag)  # Enable dragging
        else:
            self.graphics_view.setDragMode(QGraphicsView.NoDrag)  # Disable dragging

    def display_tiff_image(self, filepath):
        """Load the TIFF image and display it on the scene."""
        # Open the TIFF file with rasterio
        with rasterio.open(filepath) as dataset:
            # Read the image data as a numpy array (assuming 3 bands for RGB)
            tiff_image_data = dataset.read()

            # Get the width, height, and number of bands
            height, width = tiff_image_data.shape[1], tiff_image_data.shape[2]

            # Ensure it's an RGB image with 3 bands
            if tiff_image_data.shape[0] == 3:
                # Stack the bands (assuming the data is in the format (band, height, width))
                rgb_array = tiff_image_data.transpose(1, 2, 0)  # Shape: (height, width, bands)

                # Convert the numpy array to bytes
                rgb_bytes = rgb_array.tobytes()

                # Create QImage from the RGB data
                qimage = QImage(
                    rgb_bytes, width, height, 3 * width, QImage.Format_RGB888
                )

                # Convert QImage to QPixmap
                pixmap = QPixmap.fromImage(qimage)

                # Create a QGraphicsPixmapItem and add it to the scene
                self.pixmap_item = QGraphicsPixmapItem(pixmap)
                self.graphics_scene.addItem(self.pixmap_item)

                # Get pixel coordinates of the lat/lon and draw the marker
                x_pixel, y_pixel = self.lat_lon_to_pixel(dataset, self.lat, self.lon)
                self.draw_marker(x_pixel, y_pixel)
            else:
                raise ValueError("Unsupported image format: expecting an RGB image with 3 bands")

    @staticmethod
    def lat_lon_to_pixel(dataset, lat, lon):
        """Convert latitude and longitude to pixel coordinates using the GeoTIFF's transform matrix."""
        # Convert lat/lon to pixel row/col using dataset's transform
        row, col = rowcol(dataset.transform, lon, lat)
        return col, row

    def draw_marker(self, x_pixel, y_pixel):
        """Draw a target marker (bullseye) at the given pixel coordinates on the image."""
        # Draw the main target marker
        pixmap_copy = self.pixmap_item.pixmap().copy()
        painter = QPainter(pixmap_copy)

        # Colors for the target marker
        outer_color = QColor(13, 13, 13)  # Dark gray for the outer ring
        inner_color = QColor(255, 0, 0)  # Red for the inner circle

        # Draw outer ring
        outer_size = 14
        painter.setPen(outer_color)
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(x_pixel - outer_size // 2, y_pixel - outer_size // 2, outer_size, outer_size)

        # Draw inner circle
        inner_size = 8
        painter.setPen(inner_color)
        painter.setBrush(inner_color)
        painter.drawEllipse(x_pixel - inner_size // 2, y_pixel - inner_size // 2, inner_size, inner_size)

        painter.end()
        self.pixmap_item.setPixmap(pixmap_copy)
        self.graphics_scene.update()

        # Get the size of the QGraphicsView to compute the available space
        view_width = self.graphics_view.viewport().width()
        view_height = self.graphics_view.viewport().height()

        # Adjust zoom factor dynamically based on widget size (optional, you can modify this)
        # This ensures that the marker point will stay within the bounds of the view even after zoom
        zoom_width_factor = view_width / (2 * outer_size)
        zoom_height_factor = view_height / (2 * outer_size)

        # Use the smaller zoom factor to avoid cutting off any part of the marker
        self.zoom_factor = min(zoom_width_factor, zoom_height_factor)

        # Apply the zoom with the calculated zoom factor
        self.graphics_view.resetTransform()
        self.graphics_view.scale(self.zoom_factor, self.zoom_factor)

        # Center the view on the marker point, ensuring the point is in the center
        self.graphics_view.centerOn(x_pixel, y_pixel)

        # Initialize the ripple effect
        self.create_ripple(x_pixel, y_pixel)

    def create_ripple(self, x_pixel, y_pixel):
        """Create an expanding ripple effect around the marker using animation."""
        self.ripple_radius = 1
        self.ripple_opacity = 255

        # Timer for ripple animation
        self.ripple_timer = QTimer(self)
        self.ripple_timer.timeout.connect(lambda: self.animate_ripple(x_pixel, y_pixel))
        self.ripple_timer.start(50)  # Update every 50 ms

    def animate_ripple(self, x_pixel, y_pixel):
        """Animate the ripple effect by increasing radius and decreasing opacity."""
        self.ripple_radius += 4
        self.ripple_opacity -= 15

        # Stop the animation when ripple is almost invisible
        if self.ripple_opacity <= 0:
            self.ripple_timer.stop()
            return

        # Redraw the pixmap with the ripple effect
        pixmap_copy = self.pixmap_item.pixmap().copy()
        painter = QPainter(pixmap_copy)

        # Draw the ripple with decreasing opacity
        ripple_color = QColor(0, 38, 153, self.ripple_opacity)  # Blue with dynamic opacity
        painter.setPen(ripple_color)
        painter.setBrush(Qt.NoBrush)  # No fill for the ripple
        painter.drawEllipse(x_pixel - self.ripple_radius // 2,
                            y_pixel - self.ripple_radius // 2,
                            self.ripple_radius,
                            self.ripple_radius)

        painter.end()
        self.pixmap_item.setPixmap(pixmap_copy)
        self.graphics_scene.update()

    def zoom_in(self):
        """Increase the zoom factor and apply the transformation."""
        self.zoom_factor *= 1.2
        self.apply_zoom()

    def zoom_out(self):
        """Decrease the zoom factor and apply the transformation."""
        self.zoom_factor /= 1.2
        self.apply_zoom()

    def apply_zoom(self):
        """Apply the zoom transformation to the QGraphicsView."""
        self.graphics_view.resetTransform()  # Reset previous transformations
        self.graphics_view.scale(self.zoom_factor, self.zoom_factor)  # Apply the new zoom factor

    def center_map(self):
        """Center the map based on the new latitude and longitude values from input fields."""
        try:
            # Retrieve the latitude and longitude from the input fields
            new_lat = float(self.lat_input.text())
            new_lon = float(self.lon_input.text())

            # Update the internal latitude and longitude values
            self.lat = new_lat
            self.lon = new_lon

            # Clear the scene and reload the map with the new center
            self.graphics_scene.clear()  # Clear the current map and marker
            self.display_tiff_image('Gangagar.tif')  # Reload TIFF and center the map
        except ValueError:
            print("Invalid latitude or longitude values entered.")

    def center_on_current_location(self):
        """Center the map on the user's current location coordinates."""
        current_lat = 13.0229983  # Example latitude
        current_lon = 77.5858031  # Example longitude

        # Update the input fields with the current location values
        self.lat_input.setText(str(current_lat))
        self.lon_input.setText(str(current_lon))

        # Update the latitude and longitude values and center the map
        self.lat = current_lat
        self.lon = current_lon
        self.graphics_scene.clear()  # Clear the current map and marker
        self.display_tiff_image('Gangagar.tif')  # Reload TIFF and center the map


def main():
    app = QtWidgets.QApplication(sys.argv)
    application = MainWindow()
    application.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
