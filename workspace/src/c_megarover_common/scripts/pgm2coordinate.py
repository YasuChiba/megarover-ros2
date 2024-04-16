import sys
import yaml
import csv
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QPushButton, QVBoxLayout, QWidget, QFileDialog, QLineEdit
from PyQt5.QtGui import QPixmap, QImage
from PIL import Image

class ImageWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.selected_points = []  # Store selected points with labels
        self.initUI()

    def initUI(self):
        # Main layout and widget
        self.widget = QWidget()
        self.setCentralWidget(self.widget)
        self.layout = QVBoxLayout()

        # Button to load PGM and YAML
        self.btn_load = QPushButton('Load PGM and YAML', self)
        self.btn_load.clicked.connect(self.loadImage)
        self.layout.addWidget(self.btn_load)

        # Label to display the image
        self.label_image = QLabel(self)
        self.layout.addWidget(self.label_image)

        # Text field for entering labels
        self.text_label = QLineEdit(self)
        self.text_label.setPlaceholderText("Enter label for selected point")
        self.layout.addWidget(self.text_label)

        # Label to display coordinates and value
        self.label_info = QLabel('Select a point', self)
        self.layout.addWidget(self.label_info)

        # Button to save selected points to CSV
        self.btn_save = QPushButton('Save Points to CSV', self)
        self.btn_save.clicked.connect(self.savePoints)
        self.layout.addWidget(self.btn_save)

        self.widget.setLayout(self.layout)
        self.setGeometry(300, 300, 350, 350)
        self.setWindowTitle('PGM Viewer with ROS Coordinates and Saving')
        self.show()

    def loadImage(self):
        # Open dialog to select the YAML file
        fname, _ = QFileDialog.getOpenFileName(self, 'Open file', '.', "YAML files (*.yaml *.yml)")
        #fname = '/home/user/workspace/maps/map.yaml'
        if fname:
            with open(fname, 'r') as file:
                self.map_data = yaml.safe_load(file)
            
            # Load the image specified in the YAML file
            image_path = self.map_data['image']
            # convert filepaths to absolute paths
            if not image_path.startswith('/'):
                image_path = '/'.join(fname.split('/')[:-1]) + '/' + image_path
            #self.image = Image.open(image_path)
            self.image = Image.open(image_path).transpose(Image.FLIP_TOP_BOTTOM)
            self.qimg = QImage(image_path)
            self.qimg = self.qimg.mirrored(True, True)  # Flip the QImage vertically
            pixmap = QPixmap.fromImage(self.qimg)
            self.label_image.setPixmap(pixmap)
            self.label_image.mousePressEvent = self.getPixel

    def getPixel(self, event):
        x = event.pos().x()
        y = event.pos().y()
        pixel_value = self.image.getpixel((x, y))
        
        # Calculate real-world coordinates
        real_x = self.map_data['origin'][0] + (self.image.width - x - 1)  * self.map_data['resolution']
        real_y = self.map_data['origin'][1] + y * self.map_data['resolution']
        #self.label_info.setText(f'Pixel: ({x},{y}) Value: {pixel_value}, World: ({real_x:.2f}, {real_y:.2f})')

        label = self.text_label.text().strip() or "No Label"
        self.selected_points.append((real_x, real_y, label))
        self.label_info.setText(f'Pixel: ({x},{self.image.height - y - 1}) Value: {pixel_value}, World: ({real_x:.2f}, {real_y:.2f}), Label: {label}')


    def savePoints(self):
        fname, _ = QFileDialog.getSaveFileName(self, 'Save file', '.', "CSV files (*.csv)")
        if fname:
            with open(fname, 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['X Coordinate', 'Y Coordinate', 'Label'])
                for point in self.selected_points:
                    writer.writerow(point)
            self.label_info.setText('Points saved successfully.')

def main():
    app = QApplication(sys.argv)
    ex = ImageWindow()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()


