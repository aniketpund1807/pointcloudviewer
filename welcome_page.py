"""
Welcome Page for 3D Bharat Design & Measurement Tool
Displays after successful login, before main application window.
"""

from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel, QPushButton, QWidget
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap, QPalette, QBrush, QFont, QIcon
import os


class WelcomePage(QDialog):
    """Welcome page shown after successful login."""
    
    def __init__(self, user_full_name="", parent=None):
        super().__init__(parent)
        self.user_full_name = user_full_name
        self.setWindowTitle("Welcome - 3D Bharat")
        self.setWindowFlags(Qt.FramelessWindowHint)
        
        # Set window icon
        icon_path = os.path.join(os.path.dirname(__file__), r"3D Bharat logo.png")
        if os.path.exists(icon_path):
            self.setWindowIcon(QIcon(icon_path))
        
        # Setup UI first
        self.setup_ui()
        
        # Show fullscreen - fits to screen automatically
        self.showFullScreen()
        
        # Set background image after showing fullscreen
        self.set_background_image()
    
    def set_background_image(self):
        """Set the background image for the welcome page."""
        bg_path = os.path.join(os.path.dirname(__file__), r"C:\Users\hp\Downloads\drone-image.png")
        
        if os.path.exists(bg_path):
            palette = QPalette()
            pixmap = QPixmap(bg_path)
            scaled_pixmap = pixmap.scaled(self.size(), Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation)
            palette.setBrush(QPalette.Window, QBrush(scaled_pixmap))
            self.setPalette(palette)
            self.setAutoFillBackground(True)
    
    def setup_ui(self):
        """Setup the welcome page UI."""
        # Main layout
        layout = QVBoxLayout(self)
        layout.setContentsMargins(50, 50, 50, 50)
        
        # Spacer to push content to center
        layout.addStretch(2)
        
        # Welcome text container with semi-transparent background
        text_container = QWidget()
        text_container.setStyleSheet("""
            QWidget {
                background-color: rgba(0, 0, 0, 0.5);
                border-radius: 20px;
                padding: 30px;
            }
        """)
        text_layout = QVBoxLayout(text_container)
        text_layout.setSpacing(0)
        text_layout.setContentsMargins(40, 15, 40, 15)
        
        # Logo above title
        logo_label = QLabel()
        logo_path = os.path.join(os.path.dirname(__file__), r"3D Bharat logo.png")
        if os.path.exists(logo_path):
            logo_pixmap = QPixmap(logo_path)
            # Scale logo to appropriate size while maintaining aspect ratio
            scaled_logo = logo_pixmap.scaled(200, 200, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            logo_label.setPixmap(scaled_logo)
        logo_label.setAlignment(Qt.AlignCenter | Qt.AlignBottom)
        logo_label.setFixedHeight(140)  # Constrains height to reduce gap
        logo_label.setStyleSheet("background-color: transparent;")
        text_layout.addWidget(logo_label)
        
        # Main title - "Welcome to 3D Bharat"
        title_label = QLabel("Welcome to 3D Bharat")
        title_label.setAlignment(Qt.AlignCenter)
        title_font = QFont("Segoe UI", 42, QFont.Bold)
        title_label.setFont(title_font)
        title_label.setStyleSheet("""
            QLabel {
                color: #00FFFF;
                background-color: transparent;
            }
        """)
        text_layout.addWidget(title_label)
        
        # Subtitle - "Precise Work Progress Monitoring"
        subtitle_label = QLabel("Precise Work Progress Monitoring")
        subtitle_label.setAlignment(Qt.AlignCenter)
        subtitle_font = QFont("Segoe UI", 22)
        subtitle_label.setFont(subtitle_font)
        subtitle_label.setStyleSheet("""
            QLabel {
                color: #FFFFFF;
                background-color: transparent;
            }
        """)
        text_layout.addWidget(subtitle_label)
        
        # User greeting (if user name available)
        if self.user_full_name:
            greeting_label = QLabel(f"Hello, {self.user_full_name}!")
            greeting_label.setAlignment(Qt.AlignCenter)
            greeting_font = QFont("Segoe UI", 16)
            greeting_label.setFont(greeting_font)
            greeting_label.setStyleSheet("""
                QLabel {
                    color: #90EE90;
                    background-color: transparent;
                }
            """)
            text_layout.addWidget(greeting_label)
        
        layout.addWidget(text_container, alignment=Qt.AlignCenter)
        
        # Spacer
        layout.addStretch(1)
        
        # Start Now Button
        start_button = QPushButton("Start Now")
        start_button.setFixedSize(300, 70)
        start_button.setCursor(Qt.PointingHandCursor)
        start_button.setFont(QFont("Segoe UI", 18, QFont.Bold))
        start_button.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #00CED1, stop:0.5 #00BFFF, stop:1 #1E90FF);
                color: white;
                border: none;
                border-radius: 30px;
                padding: 15px 40px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #40E0D0, stop:0.5 #00CED1, stop:1 #00BFFF);
            }
            QPushButton:pressed {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #008B8B, stop:0.5 #00808F, stop:1 #006994);
            }
        """)
        start_button.clicked.connect(self.accept)
        layout.addWidget(start_button, alignment=Qt.AlignCenter)
        
        # Spacer at bottom
        layout.addStretch(1)
        
        # Version/Copyright text with creative frosted glass container
        version_container = QWidget()
        version_container.setFixedWidth(1000)
        version_container.setStyleSheet("""
            QWidget {
                background-color: rgba(255, 255, 255, 0.15);
                border: 2px solid rgba(0, 255, 255, 0.5);
                border-radius: 20px;
            }
        """)
        version_layout = QVBoxLayout(version_container)
        version_layout.setContentsMargins(20, 12, 20, 12)
        
        version_label = QLabel("Version 1.0 | © 2026 Micro Integrated Semiconductor Systems Pvt. Ltd. - All Rights Reserved")
        version_label.setAlignment(Qt.AlignCenter)
        version_label.setStyleSheet("""
            QLabel {
                color: #FFFFFF;
                font-size: 18px;
                font-weight: 600;
                letter-spacing: 1px;
                background-color: transparent;
                border: none;
            }
        """)
        version_layout.addWidget(version_label)
        layout.addWidget(version_container, alignment=Qt.AlignCenter)
    
    def resizeEvent(self, event):
        """Handle resize event to scale background image."""
        super().resizeEvent(event)
        self.set_background_image()
