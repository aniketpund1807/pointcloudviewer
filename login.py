
import os
import json
import math
import random
from datetime import datetime
from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QMessageBox, QTabWidget, QWidget, QFrame, QScrollArea
)
from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF
from PyQt5.QtGui import QCursor, QPixmap, QIcon, QPainter, QColor, QRadialGradient, QLinearGradient, QPainterPath, QPen
from passlib.hash import django_pbkdf2_sha256
from database import DatabaseHandler

# Folder and file paths
USER_FOLDER = r"E:\3D_Tool\user"
LAST_LOGIN_FILE = os.path.join(USER_FOLDER, "last_login.json")
os.makedirs(USER_FOLDER, exist_ok=True)


class Particle:
    """A glowing particle for the tech animation"""
    def __init__(self, x, y, speed, angle, size, color):
        self.x = x
        self.y = y
        self.speed = speed
        self.angle = angle
        self.size = size
        self.color = color
        self.alpha = random.uniform(0.3, 1.0)
        self.alpha_speed = random.uniform(0.01, 0.03)
        self.alpha_dir = 1

    def update(self, width, height):
        # Move particle
        self.x += math.cos(self.angle) * self.speed
        self.y += math.sin(self.angle) * self.speed
        
        # Pulse alpha
        self.alpha += self.alpha_speed * self.alpha_dir
        if self.alpha >= 1.0:
            self.alpha = 1.0
            self.alpha_dir = -1
        elif self.alpha <= 0.2:
            self.alpha = 0.2
            self.alpha_dir = 1
        
        # Wrap around
        if self.x < -20:
            self.x = width + 20
        elif self.x > width + 20:
            self.x = -20
        if self.y < -20:
            self.y = height + 20
        elif self.y > height + 20:
            self.y = -20


class CircuitLine:
    """Animated circuit line for tech effect"""
    def __init__(self, start_x, start_y, length, is_horizontal, color):
        self.start_x = start_x
        self.start_y = start_y
        self.length = length
        self.is_horizontal = is_horizontal
        self.color = color
        self.progress = 0
        self.speed = random.uniform(0.005, 0.02)
        self.glow_phase = random.uniform(0, math.pi * 2)

    def update(self):
        self.progress += self.speed
        if self.progress > 1:
            self.progress = 0
        self.glow_phase += 0.05


class LoginDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Login / Register")
        self.setWindowIcon(QIcon(r"C:\Users\hp\OneDrive\Documents\milogo\MI_logo.png"))
        self.setModal(True)
        
        # Make truly fullscreen - get screen geometry
        from PyQt5.QtWidgets import QApplication, QDesktopWidget
        screen = QApplication.primaryScreen()
        screen_geometry = screen.availableGeometry()
        
        # Set frameless and position at 0,0 with full screen size
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        self.setGeometry(screen_geometry)
        self.move(0, 0)
        
        # Initialize animation elements
        self.particles = []
        self.circuit_lines = []
        self.glow_phase = 0
        self.init_particles()
        self.init_circuit_lines()
        
        # Animation timer
        self.anim_timer = QTimer(self)
        self.anim_timer.timeout.connect(self.update_animation)
        self.anim_timer.start(33)  # ~30 FPS
        
        # Setup UI
        self.setup_ui()
        
        self.logged_in_username = None
        self.logged_in_user_id = None
        self.load_last_login()

    def init_particles(self):
        """Create floating particles"""
        colors = [
            QColor(0, 200, 255),   # Cyan
            QColor(0, 150, 255),   # Blue
            QColor(100, 200, 255), # Light blue
            QColor(0, 255, 200),   # Teal
        ]
        for _ in range(50):
            self.particles.append(Particle(
                x=random.uniform(0, 1920),
                y=random.uniform(0, 1080),
                speed=random.uniform(0.3, 1.0),
                angle=random.uniform(0, math.pi * 2),
                size=random.uniform(2, 6),
                color=random.choice(colors)
            ))

    def init_circuit_lines(self):
        """Create circuit line patterns"""
        color = QColor(0, 180, 255, 100)
        # Horizontal lines
        for i in range(15):
            self.circuit_lines.append(CircuitLine(
                start_x=random.uniform(0, 1920),
                start_y=random.uniform(0, 1080),
                length=random.uniform(100, 400),
                is_horizontal=True,
                color=color
            ))
        # Vertical lines
        for i in range(15):
            self.circuit_lines.append(CircuitLine(
                start_x=random.uniform(0, 1920),
                start_y=random.uniform(0, 1080),
                length=random.uniform(100, 400),
                is_horizontal=False,
                color=color
            ))

    def update_animation(self):
        """Update all animation elements"""
        self.glow_phase += 0.02
        for p in self.particles:
            p.update(self.width(), self.height())
        for line in self.circuit_lines:
            line.update()
        self.update()

    def paintEvent(self, event):
        """Paint the animated tech background"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        
        # Dark blue gradient background
        gradient = QLinearGradient(0, 0, self.width(), self.height())
        gradient.setColorAt(0, QColor(10, 15, 35))
        gradient.setColorAt(0.5, QColor(15, 25, 55))
        gradient.setColorAt(1, QColor(10, 20, 45))
        painter.fillRect(self.rect(), gradient)
        
        # Draw circuit lines
        for line in self.circuit_lines:
            glow = 0.3 + 0.7 * abs(math.sin(line.glow_phase))
            color = QColor(0, 180, 255, int(80 * glow))
            pen = QPen(color, 1)
            painter.setPen(pen)
            
            if line.is_horizontal:
                end_x = line.start_x + line.length * line.progress
                painter.drawLine(int(line.start_x), int(line.start_y), 
                               int(end_x), int(line.start_y))
                # Glowing dot at end
                if line.progress > 0.1:
                    dot_color = QColor(0, 220, 255, int(200 * glow))
                    painter.setBrush(dot_color)
                    painter.setPen(Qt.NoPen)
                    painter.drawEllipse(QPointF(end_x, line.start_y), 3, 3)
            else:
                end_y = line.start_y + line.length * line.progress
                painter.drawLine(int(line.start_x), int(line.start_y),
                               int(line.start_x), int(end_y))
                if line.progress > 0.1:
                    dot_color = QColor(0, 220, 255, int(200 * glow))
                    painter.setBrush(dot_color)
                    painter.setPen(Qt.NoPen)
                    painter.drawEllipse(QPointF(line.start_x, end_y), 3, 3)
        
        # Draw floating particles with glow
        for p in self.particles:
            # Outer glow
            glow_gradient = QRadialGradient(QPointF(p.x, p.y), p.size * 3)
            glow_color = QColor(p.color)
            glow_color.setAlpha(int(50 * p.alpha))
            glow_gradient.setColorAt(0, glow_color)
            glow_color.setAlpha(0)
            glow_gradient.setColorAt(1, glow_color)
            painter.setBrush(glow_gradient)
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(QPointF(p.x, p.y), p.size * 3, p.size * 3)
            
            # Core
            core_color = QColor(p.color)
            core_color.setAlpha(int(255 * p.alpha))
            painter.setBrush(core_color)
            painter.drawEllipse(QPointF(p.x, p.y), p.size, p.size)
        
        # Draw corner decorations (tech frame effect)
        self.draw_corner_decorations(painter)
        
        painter.end()

    def draw_corner_decorations(self, painter):
        """Draw futuristic corner decorations"""
        glow = 0.5 + 0.5 * abs(math.sin(self.glow_phase))
        color = QColor(0, 200, 255, int(150 * glow))
        pen = QPen(color, 2)
        painter.setPen(pen)
        
        corner_size = 50
        margin = 30
        w, h = self.width(), self.height()
        
        # Top-left corner
        painter.drawLine(margin, margin, margin + corner_size, margin)
        painter.drawLine(margin, margin, margin, margin + corner_size)
        
        # Top-right corner
        painter.drawLine(w - margin, margin, w - margin - corner_size, margin)
        painter.drawLine(w - margin, margin, w - margin, margin + corner_size)
        
        # Bottom-left corner
        painter.drawLine(margin, h - margin, margin + corner_size, h - margin)
        painter.drawLine(margin, h - margin, margin, h - margin - corner_size)
        
        # Bottom-right corner
        painter.drawLine(w - margin, h - margin, w - margin - corner_size, h - margin)
        painter.drawLine(w - margin, h - margin, w - margin, h - margin - corner_size)

    def setup_ui(self):
        """Setup the login UI on top of the animated background"""
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        
        # Floating close button at top right corner
        self.close_btn = QPushButton("✕", self)
        self.close_btn.setFixedSize(45, 45)
        self.close_btn.setCursor(QCursor(Qt.PointingHandCursor))
        self.close_btn.setStyleSheet("""
            QPushButton {
                background: rgba(255, 255, 255, 0.1);
                border: none;
                border-radius: 22px;
                color: rgba(255, 255, 255, 0.7);
                font-size: 20px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: rgba(255, 80, 80, 0.6);
                color: white;
            }
            QPushButton:pressed {
                background: rgba(255, 50, 50, 0.8);
            }
        """)
        self.close_btn.clicked.connect(self.close_application)
        # Position at top right corner
        self.close_btn.move(self.width() - 65, 20)
        self.close_btn.raise_()  # Ensure button is on top
        
        # Center container
        main_layout.addStretch(1)
        
        center_h_layout = QHBoxLayout()
        center_h_layout.addStretch(1)
        
        # Main card container - glassmorphic style
        self.card = QFrame()
        self.card.setFixedSize(900, 600)
        self.card.setStyleSheet("""
            QFrame {
                background: rgba(10, 20, 40, 0.85);
                border: none;
                border-radius: 15px;
            }
        """)
        
        card_layout = QHBoxLayout(self.card)
        card_layout.setContentsMargins(0, 0, 0, 0)
        card_layout.setSpacing(0)
        
        # LEFT SIDEBAR
        left_sidebar = QWidget()
        left_sidebar.setFixedWidth(320)
        left_sidebar.setStyleSheet("background: transparent;")
        sidebar_layout = QVBoxLayout(left_sidebar)
        sidebar_layout.setContentsMargins(30, 40, 30, 40)
        sidebar_layout.setSpacing(20)
        
        # Welcome title
        title = QLabel("Welcome")
        title.setStyleSheet("""
            color: #00C8FF; 
            font-size: 38px; 
            font-weight: bold; 
            background: transparent;
        """)
        title.setAlignment(Qt.AlignCenter)
        sidebar_layout.addWidget(title)
        
        # Logo
        self.logo_label = QLabel()
        self.logo_label.setAlignment(Qt.AlignCenter)
        self.logo_label.setFixedSize(140, 140)
        self.logo_label.setStyleSheet("background: transparent;")
        logo_path = r"C:\Users\hp\OneDrive\Documents\milogo\MI_logo.png"
        pixmap = QPixmap(logo_path)
        if not pixmap.isNull():
            self.logo_label.setPixmap(pixmap.scaled(140, 140, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        else:
            self.logo_label.setText("LOGO")
            self.logo_label.setStyleSheet("""
                color: #00C8FF; font-size: 24px; font-weight: bold;
                background: rgba(0, 200, 255, 0.1); border-radius: 70px;
                border: none;
            """)
        sidebar_layout.addWidget(self.logo_label, alignment=Qt.AlignCenter)
        
        # App name
        app_name = QLabel("Micro Integrated Semiconductor \n Systems Pvt. Ltd.")
        app_name.setStyleSheet("color: white; font-size: 24px; font-weight: bold; background: transparent;")
        app_name.setAlignment(Qt.AlignCenter)
        sidebar_layout.addWidget(app_name)
        
        # Subtitle
        subtitle = QLabel("3D Bharat")
        subtitle.setStyleSheet("color: rgba(0, 200, 255, 0.8); font-size: 18px; background: transparent;")
        subtitle.setAlignment(Qt.AlignCenter)
        sidebar_layout.addWidget(subtitle)
        
        # Decorative line
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet("background: rgba(0, 200, 255, 0.2); max-height: 1px; border: none;")
        sidebar_layout.addWidget(line)
        
        # Tagline
        tagline = QLabel("Design & Measurement Tool")
        tagline.setStyleSheet("color: rgba(255, 255, 255, 0.6); font-size: 20px; background: transparent;")
        tagline.setAlignment(Qt.AlignCenter)
        sidebar_layout.addWidget(tagline)
        
        sidebar_layout.addStretch(1)
        
        # RIGHT CONTENT
        right_frame = QWidget()
        right_frame.setStyleSheet("background: transparent;")
        right_layout = QVBoxLayout(right_frame)
        right_layout.setContentsMargins(20, 20, 30, 20)
        
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
            QTabWidget::pane { 
                border: none; 
                background: rgba(0, 50, 80, 0.3); 
                border-radius: 10px;
                padding: 10px;
            }
            QTabBar::tab { 
                padding: 12px 25px; 
                margin-right: 5px; 
                font-size: 14px; 
                font-weight: bold;
                background: rgba(0, 100, 150, 0.3); 
                color: #00C8FF; 
                border-top-left-radius: 8px;
                border-top-right-radius: 8px; 
                min-width: 100px;
                border: none;
            }
            QTabBar::tab:selected { 
                background: rgba(0, 150, 200, 0.4);
            }
            QTabBar::tab:hover { 
                background: rgba(0, 150, 200, 0.3);
            }
        """)
        self.tabs.addTab(self.create_login_tab(), "Login")
        self.tabs.addTab(self.create_register_tab(), "Register")
        right_layout.addWidget(self.tabs)
        
        card_layout.addWidget(left_sidebar)
        card_layout.addWidget(right_frame, stretch=1)
        
        center_h_layout.addWidget(self.card)
        center_h_layout.addStretch(1)
        
        main_layout.addLayout(center_h_layout)
        main_layout.addStretch(1)

    def load_last_login(self):
        if not os.path.exists(LAST_LOGIN_FILE):
            return
        try:
            with open(LAST_LOGIN_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
                username = data.get("username", "")
                if username:
                    self.login_username.setText(username)
                    self.tabs.setCurrentIndex(0)
        except:
            pass

    def save_last_login(self, username):
        try:
            data = {"username": username, "saved_at": datetime.now().isoformat()}
            with open(LAST_LOGIN_FILE, 'w', encoding='utf-8') as f:
                json.dump(data, f)
        except:
            pass

    def toggle_login_password_visibility(self):
        """Toggle password visibility in login form"""
        if self.login_password.echoMode() == QLineEdit.Password:
            self.login_password.setEchoMode(QLineEdit.Normal)
            # Password now visible - show "hide" icon
            self.login_password_toggle.setText("\U0001F441\u200D\U0001F5E8")
        else:
            self.login_password.setEchoMode(QLineEdit.Password)
            # Password now hidden - show "show" icon
            self.login_password_toggle.setText("\U0001F441")

    def toggle_reg_password_visibility(self):
        """Toggle password visibility in register form"""
        if self.reg_password.echoMode() == QLineEdit.Password:
            self.reg_password.setEchoMode(QLineEdit.Normal)
            # Password now visible - show "hide" icon
            self.reg_password_toggle.setText("\U0001F441\u200D\U0001F5E8")
        else:
            self.reg_password.setEchoMode(QLineEdit.Password)
            # Password now hidden - show "show" icon
            self.reg_password_toggle.setText("\U0001F441")

    def toggle_reg_confirm_visibility(self):
        """Toggle confirm password visibility in register form"""
        if self.reg_confirm.echoMode() == QLineEdit.Password:
            self.reg_confirm.setEchoMode(QLineEdit.Normal)
            # Password now visible - show "hide" icon
            self.reg_confirm_toggle.setText("\U0001F441\u200D\U0001F5E8")
        else:
            self.reg_confirm.setEchoMode(QLineEdit.Password)
            # Password now hidden - show "show" icon
            self.reg_confirm_toggle.setText("\U0001F441")

    def create_login_tab(self):
        widget = QWidget()
        widget.setStyleSheet("background: transparent;")
        layout = QVBoxLayout(widget)
        layout.setSpacing(20)
        layout.setContentsMargins(25, 30, 25, 30)

        # Title
        title = QLabel("<h2 style='color:#00C8FF;'>Login to Your Account</h2>")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("background: transparent;")
        layout.addWidget(title)

        # Description
        desc = QLabel("Enter your credentials to access the application")
        desc.setStyleSheet("color: rgba(255,255,255,0.5); font-size: 12px; background: transparent;")
        desc.setAlignment(Qt.AlignCenter)
        layout.addWidget(desc)

        layout.addSpacing(10)

        # Input field style
        input_style = """
            QLineEdit {
                font-size: 14px; 
                padding: 12px 14px; 
                background: rgba(0, 50, 80, 0.5); 
                border: none; 
                border-radius: 8px;
                color: white;
            }
            QLineEdit:focus {
                background: rgba(0, 60, 100, 0.6);
            }
            QLineEdit::placeholder {
                color: rgba(255,255,255,0.3);
            }
        """

        # Username
        layout.addWidget(QLabel("Username:", styleSheet="color:rgba(255,255,255,0.8); font-size:18px; background:transparent;"))
        self.login_username = QLineEdit(placeholderText="Enter your username")
        self.login_username.setStyleSheet(input_style)
        layout.addWidget(self.login_username)

        # Password with toggle
        layout.addWidget(QLabel("Pin:", styleSheet="color:rgba(255,255,255,0.8); font-size:18px; background:transparent;"))
        
        # Create a frame to hold password input and toggle button
        password_frame = QFrame()
        password_frame.setStyleSheet("""
            QFrame {
                background: rgba(0, 50, 80, 0.5);
                border: none;
                border-radius: 8px;
            }
        """)
        password_frame_layout = QHBoxLayout(password_frame)
        password_frame_layout.setContentsMargins(0, 0, 8, 0)
        password_frame_layout.setSpacing(0)
        
        self.login_password = QLineEdit(placeholderText="Enter your Pin")
        self.login_password.setEchoMode(QLineEdit.Password)
        self.login_password.setStyleSheet("""
            QLineEdit {
                font-size: 14px;
                padding: 12px 14px;
                background: transparent;
                border: none;
                color: white;
            }
            QLineEdit::placeholder {
                color: rgba(255,255,255,0.3);
            }
        """)
        password_frame_layout.addWidget(self.login_password)
        
        self.login_password_toggle = QPushButton("\U0001F441")  # Eye icon - click to show password
        self.login_password_toggle.setFixedSize(36, 36)
        self.login_password_toggle.setCursor(QCursor(Qt.PointingHandCursor))
        self.login_password_toggle.setStyleSheet("""
            QPushButton {
                background: transparent;
                border: none;
                font-size: 18px;
                color: rgba(0, 200, 255, 0.7);
            }
            QPushButton:hover { color: #00C8FF; }
        """)
        self.login_password_toggle.clicked.connect(self.toggle_login_password_visibility)
        password_frame_layout.addWidget(self.login_password_toggle)
        layout.addWidget(password_frame)

        layout.addSpacing(15)

        # Login button
        btn_layout = QHBoxLayout()
        btn_layout.addStretch()
        login_btn = QPushButton("  Login  ")
        login_btn.setCursor(QCursor(Qt.PointingHandCursor))
        login_btn.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #0080A0, stop:1 #00B4D8);
                color: white; 
                padding: 12px 45px; 
                font-size: 15px; 
                font-weight: bold;
                border-radius: 8px;
                border: none;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #00A0C0, stop:1 #00D4F8);
            }
            QPushButton:pressed {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #006080, stop:1 #0094B8);
            }
        """)
        login_btn.clicked.connect(self.do_login)
        btn_layout.addWidget(login_btn)
        btn_layout.addStretch()
        layout.addLayout(btn_layout)

        # Register link
        register_link = QLabel('<a href="#" style="color:#00C8FF;">New here? <strong>Create an account</strong></a>')
        register_link.setStyleSheet("background: transparent;")
        register_link.setAlignment(Qt.AlignCenter)
        register_link.linkActivated.connect(lambda: self.tabs.setCurrentIndex(1))
        layout.addWidget(register_link)

        layout.addStretch()
        return widget

    def create_register_tab(self):
        # Create scroll area for register form
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("""
            QScrollArea { background: transparent; border: none; }
            QScrollBar:vertical { background: rgba(0, 50, 80, 0.3); width: 8px; border-radius: 4px; }
            QScrollBar::handle:vertical { background: rgba(0, 200, 255, 0.5); border-radius: 4px; }
        """)
        
        widget = QWidget()
        widget.setStyleSheet("background: transparent;")
        layout = QVBoxLayout(widget)
        layout.setSpacing(12)
        layout.setContentsMargins(25, 20, 25, 20)

        # Title
        title = QLabel("<h2 style='color:#00C8FF;'>Create New Account</h2>")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("background: transparent;")
        layout.addWidget(title)

        # Input field style
        input_style = """
            QLineEdit {
                font-size: 13px; 
                padding: 10px 12px; 
                background: rgba(0, 50, 80, 0.5); 
                border: none; 
                border-radius: 6px;
                color: white;
            }
            QLineEdit:focus {
                background: rgba(0, 60, 100, 0.6);
            }
            QLineEdit::placeholder { color: rgba(255,255,255,0.3); }
        """

        password_input_style = input_style.replace("padding: 10px 12px;", "padding: 10px 40px 10px 12px;")

        self.reg_full_name = QLineEdit(placeholderText="Enter full name")
        self.reg_email = QLineEdit(placeholderText="Enter email address")
        self.reg_username = QLineEdit(placeholderText="Choose a username")
        self.reg_mobile = QLineEdit(placeholderText="Enter mobile number")
        self.reg_password = QLineEdit(placeholderText="Create a password")
        self.reg_confirm = QLineEdit(placeholderText="Confirm your password")

        # Regular fields
        for label_text, field in [("Full Name:", self.reg_full_name), ("Email:", self.reg_email),
                                   ("Username:", self.reg_username), ("Mobile Number:", self.reg_mobile)]:
            layout.addWidget(QLabel(label_text, styleSheet="color:rgba(255,255,255,0.8); font-size:12px; background:transparent;"))
            field.setStyleSheet(input_style)
            layout.addWidget(field)

        # Password with toggle
        layout.addWidget(QLabel("Password:", styleSheet="color:rgba(255,255,255,0.8); font-size:12px; background:transparent;"))
        pw_frame = QFrame()
        pw_frame.setStyleSheet("""
            QFrame {
                background: rgba(0, 50, 80, 0.5);
                border: none;
                border-radius: 6px;
            }
        """)
        pw_frame_layout = QHBoxLayout(pw_frame)
        pw_frame_layout.setContentsMargins(0, 0, 6, 0)
        pw_frame_layout.setSpacing(0)
        self.reg_password.setEchoMode(QLineEdit.Password)
        self.reg_password.setStyleSheet("""
            QLineEdit {
                font-size: 13px;
                padding: 10px 12px;
                background: transparent;
                border: none;
                color: white;
            }
            QLineEdit::placeholder { color: rgba(255,255,255,0.3); }
        """)
        pw_frame_layout.addWidget(self.reg_password)
        self.reg_password_toggle = QPushButton("\U0001F441")  # Eye icon - click to show password
        self.reg_password_toggle.setFixedSize(32, 32)
        self.reg_password_toggle.setCursor(QCursor(Qt.PointingHandCursor))
        self.reg_password_toggle.setStyleSheet("QPushButton { background: transparent; border: none; font-size: 16px; color: rgba(0, 200, 255, 0.7); } QPushButton:hover { color: #00C8FF; }")
        self.reg_password_toggle.clicked.connect(self.toggle_reg_password_visibility)
        pw_frame_layout.addWidget(self.reg_password_toggle)
        layout.addWidget(pw_frame)

        # Confirm password with toggle
        layout.addWidget(QLabel("Confirm Password:", styleSheet="color:rgba(255,255,255,0.8); font-size:12px; background:transparent;"))
        confirm_frame = QFrame()
        confirm_frame.setStyleSheet("""
            QFrame {
                background: rgba(0, 50, 80, 0.5);
                border: none;
                border-radius: 6px;
            }
        """)
        confirm_frame_layout = QHBoxLayout(confirm_frame)
        confirm_frame_layout.setContentsMargins(0, 0, 6, 0)
        confirm_frame_layout.setSpacing(0)
        self.reg_confirm.setEchoMode(QLineEdit.Password)
        self.reg_confirm.setStyleSheet("""
            QLineEdit {
                font-size: 13px;
                padding: 10px 12px;
                background: transparent;
                border: none;
                color: white;
            }
            QLineEdit::placeholder { color: rgba(255,255,255,0.3); }
        """)
        confirm_frame_layout.addWidget(self.reg_confirm)
        self.reg_confirm_toggle = QPushButton("\U0001F441")  # Eye icon - click to show password
        self.reg_confirm_toggle.setFixedSize(32, 32)
        self.reg_confirm_toggle.setCursor(QCursor(Qt.PointingHandCursor))
        self.reg_confirm_toggle.setStyleSheet("QPushButton { background: transparent; border: none; font-size: 16px; color: rgba(0, 200, 255, 0.7); } QPushButton:hover { color: #00C8FF; }")
        self.reg_confirm_toggle.clicked.connect(self.toggle_reg_confirm_visibility)
        confirm_frame_layout.addWidget(self.reg_confirm_toggle)
        layout.addWidget(confirm_frame)

        layout.addSpacing(10)

        # Register button
        btn_layout = QHBoxLayout()
        btn_layout.addStretch()
        reg_btn = QPushButton("  Create Account  ")
        reg_btn.setCursor(QCursor(Qt.PointingHandCursor))
        reg_btn.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #00A060, stop:1 #00C878);
                color: white; padding: 12px 35px; font-size: 14px; font-weight: bold;
                border-radius: 8px; border: none;
            }
            QPushButton:hover { background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #00C080, stop:1 #00E898); }
            QPushButton:pressed { background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #008050, stop:1 #00A868); }
        """)
        reg_btn.clicked.connect(self.do_register)
        btn_layout.addWidget(reg_btn)
        btn_layout.addStretch()
        layout.addLayout(btn_layout)

        layout.addStretch()
        scroll.setWidget(widget)
        return scroll

    def do_register(self):
        full_name = self.reg_full_name.text().strip()
        email = self.reg_email.text().strip()
        username = self.reg_username.text().strip()
        mobile = self.reg_mobile.text().strip()
        password = self.reg_password.text()
        confirm = self.reg_confirm.text()

        if not all([full_name, email, username, mobile, password]):
            QMessageBox.warning(self, "Error", "All fields are required!")
            return
        if password != confirm:
            QMessageBox.warning(self, "Error", "Passwords do not match!")
            return
        if len(password) < 4:
            QMessageBox.warning(self, "Error", "Password must be at least 4 characters!")
            return
        if '@' not in email or '.' not in email:
            QMessageBox.warning(self, "Error", "Invalid email format!")
            return
        if len(mobile) != 10 or not mobile.isdigit():
            QMessageBox.warning(self, "Error", "Mobile number must be 10 digits!")
            return

        db = DatabaseHandler()
        if not db.connect():
            QMessageBox.critical(self, "Error", "Cannot connect to database.")
            return

        try:
            user_id = db.get_next_id('user_header_all')

            # Check username exists
            db.cursor.execute("SELECT 1 FROM user_header_all WHERE user_username = %s LIMIT 1", (username,))
            if db.cursor.fetchone():
                QMessageBox.warning(self, "Error", "Username already exists!")
                return

            hashed_pw = django_pbkdf2_sha256.hash(password)
            now = datetime.now()
            future = datetime(9999, 12, 31, 23, 59, 59)

            query = """
            INSERT INTO user_header_all (
                user_id, user_full_name, user_email, user_username, user_password,
                user_mobile_no, user_type, status, user_inserted_on, valid_till,
                permitted_operation, mobile_token
            ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
            """

            values = (
                user_id, full_name, email, username, hashed_pw,
                mobile, 1, 1, now, future,
                1, ''
            )

            db.cursor.execute(query, values)
            db.connection.commit()

            #QMessageBox.information(self, "Success", f"User '{username}' registered successfully!\nPlease login with your credentials.")
            self.reg_full_name.clear()
            self.reg_email.clear()
            self.reg_username.clear()
            self.reg_mobile.clear()
            self.reg_password.clear()
            self.reg_confirm.clear()
            self.tabs.setCurrentIndex(0)

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Registration failed:\n{str(e)}")
        finally:
            db.disconnect()

    def do_login(self):
        username = self.login_username.text().strip()
        password = self.login_password.text()

        if not username or not password:
            QMessageBox.warning(self, "Error", "Please enter username and password!")
            return

        db = DatabaseHandler()
        if not db.connect():
            QMessageBox.critical(self, "Error", "Cannot connect to database.")
            return

        try:
            # Query database for user verification with all details
            db.cursor.execute(
                "SELECT user_id, user_full_name, user_email, user_username, user_password, "
                "user_mobile_no, user_type, status, user_inserted_on, valid_till "
                "FROM user_header_all WHERE user_username = %s AND status = 1",
                (username,)
            )
            user = db.cursor.fetchone()

            # Verify username and password
            if not user:
                QMessageBox.warning(self, "Login Failed", "Invalid username or password!")
                return

            if not django_pbkdf2_sha256.verify(password, user['user_password']):
                QMessageBox.warning(self, "Login Failed", "Invalid username or password!")
                return

            # ========== LOGIN SUCCESSFUL ==========
            user_id = user['user_id']
            self.logged_in_username = username
            self.logged_in_user_id = user_id  # Store user_id for worksheet folder creation

            # ========== CREATE USER FOLDER AND CONFIG FILE ==========
            user_folder = os.path.join(USER_FOLDER, user_id)
            
            # Create user folder if it doesn't exist
            if not os.path.exists(user_folder):
                os.makedirs(user_folder, exist_ok=True)
                print(f"Created user folder: {user_folder}")
            # else:
            #     print(f"User folder already exists: {user_folder}")

            # Create/Update user_config.txt with all user details
            user_config_file = os.path.join(user_folder, "user_config.txt")
            now = datetime.now()

            user_config_data = {
                "user_id": user_id,
                "full_name": user['user_full_name'],
                "email": user['user_email'],
                "username": user['user_username'],
                "mobile_number": user['user_mobile_no'],
                "user_type": user['user_type'],
                "status": user['status'],
                "registered_on": user['user_inserted_on'].isoformat() if user['user_inserted_on'] else None,
                "last_login": now.isoformat(),
                "valid_till": user['valid_till'].isoformat() if user['valid_till'] else None
            }

            try:
                with open(user_config_file, 'w', encoding='utf-8') as f:
                    json.dump(user_config_data, f, indent=4, ensure_ascii=False)
                # print(f"Created/Updated user config file: {user_config_file}")
            except Exception as file_error:
                print(f"Warning: Could not create user config file: {file_error}")

            # Update last_login.json
            self.save_last_login(username)

            #QMessageBox.information(self, "Success", f"Welcome {user['user_full_name']}!")
            self.accept()

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Login error:\n{str(e)}")
        finally:
            db.disconnect()

    def close_application(self):
        """Close the entire application when X button is clicked"""
        from PyQt5.QtWidgets import QApplication
        QApplication.quit()

    def resizeEvent(self, event):
        """Reposition the close button when window resizes"""
        super().resizeEvent(event)
        if hasattr(self, 'close_btn'):
            self.close_btn.move(self.width() - 65, 20)
            self.close_btn.raise_()