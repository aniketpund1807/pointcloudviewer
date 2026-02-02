# Application_UI.py
import os
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QLabel, QProgressBar, QWidget, QGroupBox, 
    QFrame, QPushButton, QSizePolicy, QTextEdit, QCheckBox, QScrollArea, QSlider

)
from PyQt5.QtCore import Qt, QByteArray, QSize, QRectF, QTimer, QPoint
from PyQt5.QtGui import QPixmap, QPainter, QIcon, QFont
from PyQt5.QtSvg import QSvgRenderer

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.ticker as ticker

# VTK imports
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from vtkmodules.vtkRenderingCore import (vtkRenderer)
from vtkmodules.vtkCommonColor import vtkNamedColors

# =====================================================================================================================================
#                                               ***  CLASS - Appilcation UI Constructor ***
# =====================================================================================================================================
class ApplicationUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.point_cloud = None
        self.vtk_widget = None
        self.renderer = None
        self.message_section = None
        self.message_text = None
        self.message_visible = False
        self.canvas = None
        self.ax = None
        self.scale_ax = None # For scale graph
        self.start_point = None
        self.end_point = None
        self.total_distance = 100.0
        self.original_total_distance = self.total_distance
        self.zero_line_actor = None
        self.zero_start_actor = None
        self.zero_end_actor = None
        self.zero_graph_line = None
        self.zero_line_set = False
        self.zero_start_point = None
        self.zero_end_point = None
        self.zero_start_km = None
        self.zero_start_chain = None
        self.zero_end_km = None
        self.zero_end_chain = None
        self.zero_interval = None
        self.zero_physical_dist = 0.0
        self.zero_start_z = 0.0 # Reference zero elevation (Z of Point_1)
        self.drawing_zero_line = False
        self.zero_points = []
        self.temp_zero_actors = []
        self.line_types = {
            'construction': {'color': 'red', 'polylines': [], 'artists': []},
            'surface': {'color': 'green', 'polylines': [], 'artists': []},
            'zero': {'color':'purple', 'polylines': [], 'artists':[]},
            'road_surface': {'color': 'blue', 'polylines': [], 'artists': []},
            'deck_line': {'color': 'blue', 'polylines': [], 'artists': []},
            'projection_line': {'color': 'green', 'polylines': [], 'artists': []},
            'construction_dots': {'color': 'red', 'polylines': [], 'artists': []},
            'material': {'color': 'orange', 'polylines': [], 'artists': []}
        }
        self.active_line_type = None
        self.current_points = []
        self.current_artist = None
        self.cid_click = None
        self.cid_key = None

# --------------------------------------------------------------------------------------------------------------------------------
        self.PENCIL_SVG = """<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24"
        viewBox="0 0 24 24" fill="none" stroke="white" stroke-width="2"
        stroke-linecap="round" stroke-linejoin="round">
        <path d="M12 20h9"/> <path d="M16.5 3.5a2.121 2.121 0 0 1 3 3L7 19l-4 1 1-4L16.5 3.5z"/>
        </svg>"""

# --------------------------------------------------------------------------------------------------------------------------------
        self.svg_left = b"""<svg width="30" height="30" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M15 18L9 12L15 6" stroke="white" stroke-width="6" stroke-linecap="round" stroke-linejoin="round"/>
        </svg>"""
        self.svg_right = b"""<svg width="30" height="30" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M9 6L15 12L9 18" stroke="white" stroke-width="6" stroke-linecap="round" stroke-linejoin="round"/>
        </svg>"""
# --------------------------------------------------------------------------------------------------------------------------------
        # Initialization for the saving the line actor, points actor
        self.measurement_actors = []
        self.measurement_points = []
        self.point_cloud = None
        self.point_cloud_actor = None
        # Colors
        self.colors = vtkNamedColors()
        self.measurement_active = True
        self.current_measurement = None
        self.measurement_started = False
        self.main_line_actor = None
        self.point_a_actor = None
        self.point_b_actor = None

        self.current_vertical_points = []        # stores the two points of the current vertical line
        self.is_presized_mode = False
        # Initialize actors for horizontal line measurement
        self.horizontal_line_actor = None
        self.point_p_actor = None
        self.point_q_actor = None
        self.horizontal_distance_label_actor = None
        # To save and load actors
        self.vertical_points = [] # for vertical line
        self.horizontal_points = [] # for horizontal line
        # For polygon
        self.polygon_points = []
        self.polygon_actors = []
        self.vertical_height_meters = 0.0 # Store vertical height
        self.distance_label_actor = None # Store distance label actor
        self.polygon_area_meters = 0.0 # Store polygon Surface Area
        self.horizontal_length_meters = 0.0 # Store horizontal length
        self.polygon_perimeter_meters = 0.0
        self.polygon_volume_meters = 0.0
        self.polygon_outer_surface_meters = 0.0
        self.presized_volume = 0.0
        self.presized_outer_surface = 0.0
        # NEW: View control states
        self.freeze_view = False # Track if view is frozen
        self.plotting_active = True # Track if plotting is active
        self.preview_actors = []
        self.all_graph_lines = []
        self.redo_stack = []
        self.current_redo_points = []

        # Initialize the list to track items
        self.material_items = []  # Important: add this in __init__ or here

        # Add this new variable to store point labels
        self.point_labels = []  # For storing point label annotations
        self.current_point_labels = []  # For current drawing session

        self.three_D_layers_layout = None
        self.two_D_layers_layout = None

        # ADD THESE TWO LINES
        self.three_D_frame = None   # Will hold the 3D Layers frame
        self.two_D_frame = None     # Will hold the 2D Layers frame

        self.current_worksheet_name = None
        self.current_project_name = None

        # Worksheet display area (top of left panel)
        self.worksheet_display = QGroupBox("Current Worksheet")

        self.worksheet_display.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 2px solid #7B1FA2;
                border-radius: 10px;
                margin: 0px;           /* ← THIS was 10px → creates the gap */
                margin-top: 20px;      /* ← Keeps the title nicely spaced above the border */
                padding: 10px 10px 10px 10px;
                background-color: rgba(225, 190, 231, 0.3);
                color: #4A148C;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 10px;
                padding: 0 5px;
                background-color: white;           /* optional: makes title background clean */
            }
        """)
        worksheet_info_layout = QVBoxLayout(self.worksheet_display)
        self.worksheet_info_label = QLabel("No worksheet loaded")
        self.worksheet_info_label.setWordWrap(True)
        self.worksheet_info_label.setAlignment(Qt.AlignCenter)
        self.worksheet_info_label.setStyleSheet("font-size: 13px; color: #333;")
        worksheet_info_layout.addWidget(self.worksheet_info_label)
        self.worksheet_display.setVisible(False)

        self.initUI()
        self.create_progress_bar()

        # Add curve-related attributes here:
        self.curve_annotation = None
        self.curve_arrow_annotation = None
        self.curve_pick_id = None
        self.curve_annotation_x_pos = None
        self.current_curve_config = {'outer_curve': False, 'inner_curve': False, 'angle': 0.0}

        # To store the last point of the surface line (for curve annotation)
        self.last_surface_point_x = None

        self.curve_annotation_x_pos = None  # To remember where it is placed

        # ADD THESE NEW VARIABLES (near other self. variables)
        self.curve_pick_id = None  # For clickable annotation
        self.curve_annotation = None
        self.curve_arrow_annotation = None
        self.current_curve_config = {'outer_curve': False, 'inner_curve': False, 'angle': 0.0}
        self.curve_active = False  # NEW: Track if a curve is currently active

        # Add these attributes
        self.current_mode = None  # 'road' or 'bridge'
        self.road_lines_data = {}  # Store road lines when switching to bridge
        self.bridge_lines_data = {}  # Store bridge lines when switching to road

        # Add these to the __init__ method
        self.road_lines_data = {
            'construction': {'polylines': [], 'artists': []},
            'surface': {'polylines': [], 'artists': []},
            'road_surface': {'polylines': [], 'artists': []},
            'zero': {'polylines': [], 'artists': []}
        }

        self.bridge_lines_data = {
            'deck_line': {'polylines': [], 'artists': []},
            'projection_line': {'polylines': [], 'artists': []},
            'construction_dots': {'polylines': [], 'artists': []},
            'zero': {'polylines': [], 'artists': []}
        }

        self.current_mode = None 

        # In the __init__ method, add these attributes:
        self.last_click_time = 0
        self.double_click_threshold = 0.5

        self.construction_dot_artists = [] 

        self.material_items = []

        # Create a layout for material items if it doesn't exist
        if not hasattr(self, 'material_items_layout'):
            self.material_items_layout = QVBoxLayout()
            self.material_items_layout.setContentsMargins(0, 0, 0, 0)
            self.material_items_layout.setSpacing(5)
            
            # Create a widget to hold the material items
            material_items_widget = QWidget()
            material_items_widget.setLayout(self.material_items_layout)

# ========================================================================================================================================================
#                                                           *** Application UI Function ***
# =========================================================================================================================================================
    def initUI(self):
        self.setWindowTitle('3D Bharat Design & Measurement Tool')
        self.setWindowIcon(QIcon(r"C:\Users\hp\OneDrive\Documents\milogo\MI_logo.png"))
        self.setGeometry(100, 100, 1200, 800)
        
        # Main vertical layout for the entire window
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(2)
        
        # -----------------------------------------------------------------
        # TOP SECTION – Toolbar
        # ------------------------------------------------------------------
        top_section = QFrame()
        top_section.setFrameStyle(QFrame.Box | QFrame.Raised)
        top_section.setFixedHeight(80)
        top_section.setStyleSheet("""
            QFrame {
                border: 3px solid #8F8F8F;
                border-radius: 10px;
                background-color: #8FBFEF;
                margin-left: 3px;
                margin-right: 3px;
            }
        """)
        top_layout = QHBoxLayout(top_section)
        top_layout.setContentsMargins(10, 5, 10, 5)
        top_layout.setSpacing(15)
        top_layout.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        # Logo instead of hamburger menu
        top_logo = QLabel()
        logo_path = os.path.join(os.path.dirname(__file__), r"3D Bharat logo.png")
        if os.path.exists(logo_path):
            logo_pixmap = QPixmap(logo_path)
            scaled_logo = logo_pixmap.scaled(100, 100, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            top_logo.setPixmap(scaled_logo)
        top_logo.setStyleSheet("background: transparent; border:None;")
        top_layout.addWidget(top_logo)
        # ------------------------------------------------------------------
        # Helper: create a button with dropdown (New / Existing)
        # ------------------------------------------------------------------
        def create_dropdown_button(text, emoji, width=120):
            btn = QPushButton(f"{emoji} {text}")
            btn.setFixedWidth(width)
            btn.setFixedHeight(65)
            btn.setCheckable(True)

            # Dropdown widget (the little box that appears below)
            dropdown = QWidget()
            dropdown.setWindowFlags(Qt.Popup)
            dropdown_layout = QVBoxLayout(dropdown)
            dropdown_layout.setContentsMargins(4, 4, 4, 4)
            dropdown_layout.setSpacing(2)

            btn_new = QPushButton("New")
            btn_existing = QPushButton("Existing")
            btn_new.setFixedHeight(40)
            btn_new.setFixedWidth(100)
            btn_existing.setFixedWidth(100)
            btn_existing.setFixedHeight(40)

            dropdown_layout.addWidget(btn_new)
            dropdown_layout.addWidget(btn_existing)

            # Connect (you can change these slots to your real functions)
            btn_new.clicked.connect(lambda: self.on_dropdown_choice(btn, "New"))
            btn_existing.clicked.connect(lambda: self.on_dropdown_choice(btn, "Existing"))

            # Show/hide logic
            def toggle():
                if btn.isChecked():
                    # close any other open dropdown first
                    for other in [self.worksheet_button, self.design_button,
                                  self.construction_button, self.measurement_button,
                                  self.earthwork_button, self.setting_button]:  # <-- added new buttons
                        if other is not btn and other.isChecked():
                            other.setChecked(False)
                            other.property("dropdown").hide()
                    # position exactly under the button
                    pos = btn.mapToGlobal(QPoint(0, btn.height()))
                    dropdown.move(pos)
                    dropdown.show()
                    btn.setProperty("dropdown", dropdown)
                else:
                    dropdown.hide()

            btn.clicked.connect(toggle)

            # Close when clicking outside
            dropdown.installEventFilter(self)

            return btn, dropdown, btn_new, btn_existing
        
        # ------------------------------------------------------------------
        # Create Project Button
        # ------------------------------------------------------------------
        self.create_project_button = QPushButton("➕ \n Create Project"
                                        )
        self.create_project_button.setStyleSheet("font-weight: bold;")
        self.create_project_button.setFixedWidth(130)
        self.create_project_button.setFixedHeight(65)
        # self.create_project_button.clicked.connect(self.open_create_project_dialog)
        top_layout.addWidget(self.create_project_button)

        # ------------------------------------------------------------------
        # Worksheet button + dropdown
        # ------------------------------------------------------------------
        (self.worksheet_button, dropdown1,
        self.new_worksheet_button, self.existing_worksheet_button) = create_dropdown_button(
            "\n Worksheet", "📊", width=130)
        self.worksheet_button.setStyleSheet("font-weight: bold;")
        # self.new_worksheet_button.clicked.connect(self.open_new_worksheet_dialog)
        top_layout.addWidget(self.worksheet_button)

        # ------------------------------------------------------------------
        # Design button + dropdown
        # ------------------------------------------------------------------
        (self.design_button, dropdown2,
        self.new_design_button, self.existing_design_button) = create_dropdown_button(
            "\n Design", "📐", width=130)
        self.design_button.setStyleSheet("font-weight: bold;")
        # self.new_design_button.clicked.connect(self.open_create_new_design_layer_dialog)
        top_layout.addWidget(self.design_button)

        # ------------------------------------------------------------------
        # Construction button + dropdown
        # ------------------------------------------------------------------
        (self.construction_button, dropdown3,
        self.new_construction_button, self.existing_construction_button) = create_dropdown_button(
            "\n Construction", "🏗", width=150)
        self.construction_button.setStyleSheet("font-weight: bold;")
        # self.new_construction_button.clicked.connect(self.open_construction_layer_dialog)
        top_layout.addWidget(self.construction_button)

        # ------------------------------------------------------------------
        # Measurement button + dropdown
        # ------------------------------------------------------------------
        (self.measurement_button, dropdown4,
        self.new_measurement_button, self.existing_measurement_button) = create_dropdown_button(
            "\n Measurement", "📏", width=150)
        self.measurement_button.setStyleSheet("font-weight: bold;")
        # self.new_measurement_button.clicked.connect(self.open_measurement_dialog)
        top_layout.addWidget(self.measurement_button)


        # ------------------------------------------------------------------
        # NEW: Earthwork button + dropdown (only "Rolling")
        # ------------------------------------------------------------------
        self.earthwork_button = QPushButton("🌍 \n Earthwork")
        self.earthwork_button.setFixedWidth(130)
        self.earthwork_button.setCheckable(True)
        self.earthwork_button.setStyleSheet("font-weight: bold;")

        earthwork_dropdown = QWidget()
        earthwork_dropdown.setWindowFlags(Qt.Popup)
        earthwork_dd_layout = QVBoxLayout(earthwork_dropdown)
        earthwork_dd_layout.setContentsMargins(4, 4, 4, 4)
        earthwork_dd_layout.setSpacing(2)

        self.rolling_button = QPushButton("Rolling")
        self.rolling_button.setFixedHeight(40)
        self.rolling_button.setFixedWidth(100)
        earthwork_dd_layout.addWidget(self.rolling_button)

        def toggle_earthwork():
            if self.earthwork_button.isChecked():
                # Close other dropdowns
                for other in [self.worksheet_button, self.design_button,
                              self.construction_button, self.measurement_button,
                              self.setting_button]:
                    if other.isChecked():
                        other.setChecked(False)
                        other.property("dropdown").hide()
                pos = self.earthwork_button.mapToGlobal(QPoint(0, self.earthwork_button.height()))
                earthwork_dropdown.move(pos)
                earthwork_dropdown.show()
                self.earthwork_button.setProperty("dropdown", earthwork_dropdown)
            else:
                earthwork_dropdown.hide()

        self.earthwork_button.clicked.connect(toggle_earthwork)
        earthwork_dropdown.installEventFilter(self)

        top_layout.addWidget(self.earthwork_button)

        # ------------------------------------------------------------------
        # Other buttons (unchanged)
        # ------------------------------------------------------------------
        self.layers_button = QPushButton("📚 \n Layers")
        self.layers_button.setStyleSheet("font-weight: bold;")
        self.layers_button.setFixedWidth(100)
        self.layers_button.setFixedHeight(65)
        top_layout.addWidget(self.layers_button)

        self.load_button = QPushButton("💠 \n 3D Point Cloud")
        self.load_button.setStyleSheet("font-weight: bold;")
        self.load_button.setFixedWidth(160)
        self.load_button.setFixedHeight(65)
        # self.load_button.clicked.connect(self.load_point_cloud)
        top_layout.addWidget(self.load_button)

        self.help_button = QPushButton("❓\n Help")
        self.help_button.setStyleSheet("font-weight: bold;")
        self.help_button.setFixedWidth(100)
        self.help_button.setFixedHeight(65)
        # self.help_button.clicked.connect(self.show_help_dialog)
        top_layout.addWidget(self.help_button)

        # ------------------------------------------------------------------
        # UPDATED: Settings button with multi-level nested dropdowns
        # ------------------------------------------------------------------
        self.setting_button = QPushButton("⚙ \n Settings")
        self.setting_button.setStyleSheet("font-weight: bold;")
        self.setting_button.setFixedWidth(110)
        self.setting_button.setFixedHeight(65)
        self.setting_button.setCheckable(True)

        # Main Settings Dropdown
        settings_dropdown = QWidget()
        settings_dropdown.setWindowFlags(Qt.Popup)
        settings_layout = QVBoxLayout(settings_dropdown)
        settings_layout.setContentsMargins(4, 4, 4, 4)
        settings_layout.setSpacing(2)

        self.general_setting_btn = QPushButton("General Setting")
        self.camera_setting_btn = QPushButton("Camera Setting ▶")

        for btn in [self.general_setting_btn, self.camera_setting_btn]:
            btn.setFixedHeight(40)
            btn.setFixedWidth(160)
            settings_layout.addWidget(btn)

        # Sub-dropdown: Camera Settings (Angle, Zoom, View)
        camera_sub_dropdown = QWidget()
        camera_sub_dropdown.setWindowFlags(Qt.Popup)
        camera_sub_layout = QVBoxLayout(camera_sub_dropdown)
        camera_sub_layout.setContentsMargins(4, 4, 4, 4)
        camera_sub_layout.setSpacing(2)

        self.angle_btn = QPushButton("Angle")
        self.zoom_btn = QPushButton("Zoom")
        self.view_btn = QPushButton("View ▶")

        for btn in [self.angle_btn, self.zoom_btn, self.view_btn]:
            btn.setFixedHeight(40)
            btn.setFixedWidth(140)
            camera_sub_layout.addWidget(btn)

        # Sub-sub-dropdown: View Options (Top, Left, Right)
        view_sub_dropdown = QWidget()
        view_sub_dropdown.setWindowFlags(Qt.Popup)
        view_sub_layout = QVBoxLayout(view_sub_dropdown)
        view_sub_layout.setContentsMargins(4, 4, 4, 4)
        view_sub_layout.setSpacing(2)

        self.top_view_btn = QPushButton("Top View")
        self.left_side_btn = QPushButton("Left Side")
        self.right_side_btn = QPushButton("Right Side")

        for vbtn in [self.top_view_btn, self.left_side_btn, self.right_side_btn]:
            vbtn.setFixedHeight(40)
            vbtn.setFixedWidth(120)
            view_sub_layout.addWidget(vbtn)

        # Toggle View sub-dropdown (Top/Left/Right)
        def toggle_view_sub():
            if self.view_btn.isChecked():
                pos = self.view_btn.mapToGlobal(QPoint(self.view_btn.width(), 0))
                view_sub_dropdown.move(pos)
                view_sub_dropdown.show()
            else:
                view_sub_dropdown.hide()

        self.view_btn.setCheckable(True)
        self.view_btn.clicked.connect(toggle_view_sub)

        # Toggle Camera sub-dropdown (Angle, Zoom, View)
        def toggle_camera_sub():
            if self.camera_setting_btn.isChecked():
                pos = self.camera_setting_btn.mapToGlobal(QPoint(self.camera_setting_btn.width(), 0))
                camera_sub_dropdown.move(pos)
                camera_sub_dropdown.show()
                # Uncheck View to prevent overlap
                self.view_btn.setChecked(False)
                view_sub_dropdown.hide()
            else:
                camera_sub_dropdown.hide()
                view_sub_dropdown.hide()
                self.view_btn.setChecked(False)

        self.camera_setting_btn.setCheckable(True)
        self.camera_setting_btn.clicked.connect(toggle_camera_sub)

        # Main Settings toggle
        def toggle_settings():
            if self.setting_button.isChecked():
                # Close other main dropdowns
                for other in [self.worksheet_button, self.design_button,
                              self.construction_button, self.measurement_button,
                              self.earthwork_button]:
                    if other.isChecked():
                        other.setChecked(False)
                        if other.property("dropdown"):
                            other.property("dropdown").hide()
                pos = self.setting_button.mapToGlobal(QPoint(0, self.setting_button.height()))
                settings_dropdown.move(pos)
                settings_dropdown.show()
                self.setting_button.setProperty("dropdown", settings_dropdown)
            else:
                settings_dropdown.hide()
                camera_sub_dropdown.hide()
                view_sub_dropdown.hide()
                self.camera_setting_btn.setChecked(False)
                self.view_btn.setChecked(False)

        self.setting_button.clicked.connect(toggle_settings)
        settings_dropdown.installEventFilter(self)

        top_layout.addWidget(self.setting_button)

        top_layout.addStretch()          # push everything to the left

        # ------------------------------------------------------------------
        # USER NAME AND LOGOUT BUTTON (Right side of header)
        # ------------------------------------------------------------------
        # User full name label
        self.user_name_label = QLabel("👤 Guest")
        self.user_name_label.setStyleSheet("""
            QLabel {
                font-weight: bold;
                font-size: 14px;
                color: #2c3e50;
                padding: 5px 10px;
                background: rgba(255, 255, 255, 0.7);
                border-radius: 5px;
            }
        """)
        top_layout.addWidget(self.user_name_label)

        # Logout button (red with white text)
        self.logout_button = QPushButton("⏻ Logout")
        self.logout_button.setFixedWidth(100)
        self.logout_button.setFixedHeight(40)
        self.logout_button.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                font-weight: bold;
                font-size: 13px;
                border: none;
                border-radius: 5px;
                padding: 5px 15px;
            }
            QPushButton:hover {
                background-color: #c0392b;
            }
            QPushButton:pressed {
                background-color: #a93226;
            }
        """)
        top_layout.addWidget(self.logout_button)

        main_layout.addWidget(top_section)

        # -----------------------------------------------------------------
        # MIDDLE CONTENT AREA (Left + Right sections)
        # ------------------------------------------------------------------
        content_widget = QWidget()
        content_layout = QHBoxLayout(content_widget)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(2)

        # --------------------------------
        # LEFT SECTION – Settings
        # ------------------------------------------------------------------
        left_section = QFrame()
        left_section.setFrameStyle(QFrame.Box | QFrame.Raised)
        left_section.setStyleSheet("""
            QFrame { 
                border: 2px solid #4CAF50; 
                border-radius: 10px; 
                background-color: #E8F5E9; 
                margin: 3px;
            }
        """)
        left_section.setMinimumWidth(380)
        left_section.setMaximumWidth(420)
        self.left_layout = QVBoxLayout(left_section)
        self.left_layout.setContentsMargins(5, 7, 5, 5)
        self.left_layout.setSpacing(5)

        # ------------------------------------------------------------------
        # NEW: Mode Banner (Measurement / Design) - TOP MOST in left panel
        # ------------------------------------------------------------------
        self.mode_banner = QLabel("No Mode Active")
        self.mode_banner.setAlignment(Qt.AlignCenter)
        self.mode_banner.setVisible(False)  # Hidden until worksheet is loaded
        self.left_layout.addWidget(self.mode_banner)

        # Add worksheet display BELOW the mode banner
        self.left_layout.insertWidget(1, self.worksheet_display)  # You already have this line — keep it

        # ---------------------------------------------------------------------------
        # Merger Layers Section
        # ---------------------------------------------------------------------------
        merger_frame = QFrame()
        merger_frame.setFrameStyle(QFrame.Box | QFrame.Raised)
        merger_frame.setStyleSheet("""
            QFrame { 
                border: 2px solid #42A5F5; 
                border-radius: 10px; 
                background-color: #E3F2FD; 
                margin: 5px; 
            }
        """)
        merger_layout = QVBoxLayout(merger_frame)
        merger_frame.setVisible(False)                    # change

        merger_title = QLabel("Merger Layers")
        merger_title.setAlignment(Qt.AlignCenter)
        merger_title.setStyleSheet("""
            font-weight: bold; 
            font-size: 13px; 
            padding: 8px; 
            background-color: #E3F2FD; 
            border-radius: 5px;
        """)
        merger_layout.addWidget(merger_title)
        merger_layout.addStretch()
        
        # --------------------------------------------------------------------------- 
        # 3D Layers Section
        # ---------------------------------------------------------------------------
        self.three_D_frame = QFrame()
        self.three_D_frame.setFrameStyle(QFrame.Box | QFrame.Raised)
        self.three_D_frame.setStyleSheet("""
            QFrame { 
                border: 2px solid #42A5F5; 
                border-radius: 10px; 
                background-color: #E3F2FD; 
                margin: 5px;
            }
        """)
        three_D_layout = QVBoxLayout(self.three_D_frame)
        self.three_D_frame.setVisible(False)

        three_D_title = QLabel("3D Layers")
        three_D_title.setAlignment(Qt.AlignCenter)
        three_D_title.setStyleSheet("""
            font-weight: bold; 
            font-size: 14px; 
            padding: 8px; 
            background-color: #BBDEFB; 
            border-radius: 6px;
            color: #1565C0;
        """)
        three_D_layout.addWidget(three_D_title)

        # Scroll area for 3D layers
        three_D_scroll = QScrollArea()
        three_D_scroll.setWidgetResizable(True)
        three_D_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        three_D_scroll.setStyleSheet("background-color: white; border: none;")
        
        three_D_content = QWidget()
        self.three_D_layers_layout = QVBoxLayout(three_D_content)
        self.three_D_layers_layout.setAlignment(Qt.AlignTop)
        self.three_D_layers_layout.setSpacing(6)
        self.three_D_layers_layout.addStretch()  # Push items to top
        
        three_D_scroll.setWidget(three_D_content)
        three_D_layout.addWidget(three_D_scroll)

        # --------------------------------------------------------------------------- 
        # 2D Layers Section
        # ---------------------------------------------------------------------------
        self.two_D_frame = QFrame()
        self.two_D_frame.setFrameStyle(QFrame.Box | QFrame.Raised)
        self.two_D_frame.setStyleSheet("""
            QFrame { 
                border: 2px solid #66BB6A; 
                border-radius: 10px; 
                background-color: #E8F5E9; 
                margin: 5px;
            }
        """)
        two_D_layout = QVBoxLayout(self.two_D_frame)
        self.two_D_frame.setVisible(False)

        two_D_title = QLabel("2D Layers")
        two_D_title.setAlignment(Qt.AlignCenter)
        two_D_title.setStyleSheet("""
            font-weight: bold; 
            font-size: 14px; 
            padding: 8px; 
            background-color: #C8E6C9; 
            border-radius: 6px;
            color: #2E7D32;
        """)
        two_D_layout.addWidget(two_D_title)

        # Scroll area for 2D layers
        two_D_scroll = QScrollArea()
        two_D_scroll.setWidgetResizable(True)
        two_D_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        two_D_scroll.setStyleSheet("background-color: white; border: none;")
        
        two_D_content = QWidget()
        self.two_D_layers_layout = QVBoxLayout(two_D_content)
        self.two_D_layers_layout.setAlignment(Qt.AlignTop)
        self.two_D_layers_layout.setSpacing(6)
        self.two_D_layers_layout.addStretch()
        
        two_D_scroll.setWidget(two_D_content)
        two_D_layout.addWidget(two_D_scroll)

        # === ADD BOTH FRAMES TO LEFT PANEL ===
        self.checkboxes = self.add_layers_content()
        self.left_layout.addWidget(self.checkboxes)
        self.left_layout.addWidget(merger_frame)
        self.left_layout.addWidget(self.three_D_frame)
        self.left_layout.addWidget(self.two_D_frame)
        self.checkboxes.setVisible(False)

        # Optional: Add stretch at bottom so layers stay at top
        self.left_layout.addStretch()


        # ==================== MESSAGE SECTION (COLLAPSIBLE) ====================
        self.message_button = QPushButton("Message")
        self.message_button.setStyleSheet("""
            QPushButton { 
                background-color: #FF8A65; 
                color: white; 
                border: none; 
                padding: 8px;
                border-radius: 5px; 
                font-weight: bold; 
                font-size: 12px; 
                Margin-left: 5px;   
                Margin-right: 5px;
            }
            QPushButton:hover { background-color: #FFCCBC; }
        """)
        self.message_button.clicked.connect(self.toggle_message_section)
        self.left_layout.addWidget(self.message_button)
        
        self.message_section = QFrame()
        self.message_section.setVisible(False)
        self.message_section.setStyleSheet("""
            QFrame { 
                border: 2px solid #FF5722; 
                border-radius: 8px; 
                background-color: #FFF3E0; 
                margin: 5px 10px; 
            }
        """)
        msg_layout = QVBoxLayout(self.message_section)
        msg_title = QLabel("Terminal Output / Errors")
        msg_title.setStyleSheet("font-weight: bold; color: #D84315; padding: 5px;")
        msg_layout.addWidget(msg_title)
        
        self.message_text = QTextEdit()
        self.message_text.setReadOnly(True)
        self.message_text.setStyleSheet("""
            QTextEdit { 
                background-color: #FFF8E1; 
                border: 1px solid #FF8A65; 
                border-radius: 5px;
                font-family: Consolas, monospace; 
                font-size: 11px; 
                padding: 5px; 
            }
        """)
        self.message_text.setMinimumHeight(150)
        msg_layout.addWidget(self.message_text)
        self.left_layout.addWidget(self.message_section)

        # Reset buttons container
        self.reset_buttons_container = QWidget()
        self.reset_buttons_layout = QHBoxLayout(self.reset_buttons_container)
        self.reset_buttons_layout.setSpacing(10)
        
        self.reset_action_button = QPushButton("Reset")
        self.reset_all_button = QPushButton("Reset_All")
        
        self.reset_buttons_layout.addWidget(self.reset_action_button)
        self.reset_buttons_layout.addWidget(self.reset_all_button)
        
        self.left_layout.addWidget(self.reset_buttons_container)

        # Add left section to content layout
        content_layout.addWidget(left_section, 1)

        # ------------------------------------------------------------------
        # RIGHT SECTION (Visualization + Controls) - NOW SCROLLABLE
        # ------------------------------------------------------------------
        # Create the main container widget
        self.right_section = QWidget()
        right_layout = QVBoxLayout(self.right_section)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(2)  # Small spacing between sections
        self.right_section.setStyleSheet("""
            QWidget {
                background-color: transparent;
            }
            QFrame { 
                margin: 3px;
            }
        """)

        # Create a scroll area for the right section
        scroll_area = QScrollArea()
        scroll_area.setWidget(self.right_section)
        scroll_area.setWidgetResizable(True)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # Style the scroll area
        scroll_area.setStyleSheet("""
            QScrollArea {
                border: none;
                background: transparent;
                margin: 0px;
                padding: 0px;
            }
            QScrollBar:vertical {
                border: 1px solid #cccccc;
                background: #f0f0f0;
                width: 14px;
                margin: 0px;
                border-radius: 7px;
            }
            QScrollBar::handle:vertical {
                background: #a0a0a0;
                min-height: 30px;
                border-radius: 7px;
            }
            QScrollBar::handle:vertical:hover {
                background: #808080;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                border: none;
                background: none;
                height: 0px;
            }
        """)

        # Make sure the right_section has a minimum size to trigger scrolling
        self.right_section.setMinimumHeight(900) 

        # Store scroll_area as instance variable for event handling
        self.right_scroll_area = scroll_area

        # ------------------------------------------------------------------
        # MIDDLE SECTION – Visualization
        # ------------------------------------------------------------------
        middle_section = QFrame()
        middle_section.setFrameStyle(QFrame.Box | QFrame.Raised)
        middle_section.setSizePolicy(
            QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        )
        middle_section.setStyleSheet("""
            QFrame {
                border: 3px solid #BA68C8;
                border-radius: 10px;
                background-color: #E6E6FA;
            }
        """)
        middle_layout = QVBoxLayout(middle_section)
        middle_layout.setContentsMargins(2, 2, 2, 2)
        
        # VTK widget
        self.vtk_widget = QVTKRenderWindowInteractor(middle_section)
        self.vtk_widget.setSizePolicy(
            QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        )
        self.vtk_widget.setMinimumHeight(380)
        self.vtk_widget.installEventFilter(self)
        self.renderer = vtkRenderer()
        self.renderer.SetBackground(1, 1, 1)  # white background
        self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)
        self.interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        middle_layout.addWidget(self.vtk_widget)

        # ------------------------------------------------------------------
        # SCALE SECTION
        # ------------------------------------------------------------------
        scale_section = QFrame()
        scale_section.setFrameStyle(QFrame.Box | QFrame.Raised)
        scale_section.setStyleSheet("""
            QFrame {
                border: 2px solid #FF9800;
                border-radius: 10px;
                background-color: #FFF3E0;
            }
        """)
        scale_section.setFixedHeight(180)
        scale_layout = QVBoxLayout(scale_section)
        scale_layout.setContentsMargins(0, 0, 0, 0)

        # Volume slider
        self.volume_slider = QSlider(Qt.Horizontal)
        self.volume_slider.setRange(0, 100)
        self.volume_slider.setValue(0)
        self.volume_slider.setTickPosition(QSlider.TicksBelow)
        self.volume_slider.setTickInterval(5)
        self.volume_slider.setStyleSheet("""
            QSlider {
                padding-left: 16px;
                padding-right: 17px;
                margin: 2px 2px;
            }
            QSlider::groove:horizontal {
                border: none;
                height: 9px;
                background: #E0E0E0;
                border-radius: 3px;
                margin: 0px 0;
            }
            QSlider::sub-page:horizontal {
                background: #4CAF50;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: white;
                border: 2px solid #4CAF50;
                width: 18px;
                height: 18px;
                margin: -6px 0;
                border-radius: 9px;
            }
            QSlider::handle:horizontal:hover {
                background: #F1F8E9;
                border: 2px solid #2E7D32;
            }
        """)
        # self.volume_slider.valueChanged.connect(self.volume_changed)
        scale_layout.addWidget(self.volume_slider)

        # Scale figure
        self.scale_figure = Figure(dpi=100)
        self.scale_figure.set_size_inches(8, 1.2)
        self.scale_canvas = FigureCanvas(self.scale_figure)
        self.scale_canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.scale_canvas.setMinimumHeight(120)
        self.scale_canvas.setMaximumHeight(140)

        # Initialize the scale axes
        self.scale_ax = self.scale_figure.add_subplot(111)
        self.scale_ax.set_xlim(0, self.total_distance)
        self.scale_ax.set_ylim(0, 1.2)
        self.scale_ax.set_facecolor('#FFF3E0')
        self.scale_ax.set_xlabel('Chainage', labelpad=3)
        self.scale_ax.set_ylabel('')
        self.scale_ax.set_yticks([])
        self.scale_ax.set_yticklabels([])

        # Create initial scale line and marker
        self.scale_line, = self.scale_ax.plot([0, self.total_distance], [0.5, 0.5], 
                                            color='black', linewidth=3)
        self.scale_marker, = self.scale_ax.plot([0, 0], [0, 1], color='red', 
                                            linewidth=2, linestyle='--')

        # Set initial ticks
        self.scale_ax.set_xticks([])
        self.scale_ax.set_xticklabels([])
        self.scale_ax.tick_params(axis='x', which='both', bottom=True, labelbottom=True, pad=5)
        self.scale_ax.grid(True, axis='x', linestyle='-', alpha=0.3)
        self.scale_ax.spines['top'].set_visible(False)
        self.scale_ax.spines['right'].set_visible(False)
        self.scale_ax.spines['left'].set_visible(False)

        # Adjust layout
        self.scale_figure.tight_layout(rect=[0, 0.1, 1, 0.95])
        self.scale_canvas.draw()
        scale_layout.addWidget(self.scale_canvas)

        # HIDE THE SCALE SECTION INITIALLY
        scale_section.setVisible(False)
        self.scale_section = scale_section

        self.scale_section.installEventFilter(self)
        self.volume_slider.installEventFilter(self)
        self.scale_canvas.installEventFilter(self)

        # ------------------------------------------------------------------
        # BOTTOM SECTION – Controls
        # ------------------------------------------------------------------
        self.bottom_section = QFrame()
        self.bottom_section.setFrameStyle(QFrame.Box | QFrame.Raised)
        self.bottom_section.setSizePolicy(
            QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        )
        self.bottom_section.setStyleSheet("""
            QFrame {
                border: 3px solid #8F8F8F;
                border-radius: 10px;
                background-color: #8FBFEF;
            }
        """)
        self.bottom_section.setMinimumHeight(400)
        bottom_layout = QVBoxLayout(self.bottom_section)
        bottom_layout.setContentsMargins(0, 0, 0, 0)
        bottom_layout.setSpacing(0)
        self.bottom_section.setVisible(False)  # Hide bottom section initially
        
        # ---------------------------------------------------------------------
        # Line Section (Collapsible)
        # ---------------------------------------------------------------------
        self.line_section = QFrame()
        self.line_section.setFrameStyle(QFrame.Box | QFrame.Raised)
        self.line_section.setSizePolicy(
            QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        )
        self.line_section.setStyleSheet("""
            QFrame {
                border: 3px solid #BA68C8;
                border-radius: 10px;
                background-color: #E6E6FA;
                margin: 0px;
                padding: 0px;
            }
        """)
        self.line_section.setMinimumWidth(80)
        self.line_section.setMaximumWidth(270)
        line_layout = QVBoxLayout(self.line_section)
        line_layout.setContentsMargins(5, 5, 5, 5)  # Remove margins
        line_layout.setSpacing(5)  # Remove spacing

        # Create a container for the top bar (header)
        self.header_container = QWidget()
        self.header_container.setFixedHeight(45)  # Fixed height for header
        header_layout = QHBoxLayout(self.header_container)
        header_layout.setContentsMargins(5, 5, 5, 5)
        header_layout.setSpacing(5)

        # Collapse button (left aligned)
        self.collapse_button = QPushButton("◀")
        self.collapse_button.setFixedSize(35, 35)
        self.collapse_button.setStyleSheet("""
            QPushButton {
                background-color: #9C27B0;
                color: white;
                border: none;
                border-radius: 5px;
                font-weight: bold;
                font-size: 16px;
            }
            QPushButton:hover {
                background-color: #7B1FA2;
            }
            QPushButton:pressed {
                background-color: #6A1B9A;
            }
        """)
        self.collapse_button.clicked.connect(self.toggle_line_section)
        self.collapse_button.setCursor(Qt.PointingHandCursor)
        self.collapse_button.setToolTip("Close Line Section")
        header_layout.addWidget(self.collapse_button)

        # Stretch to push undo/redo to right
        header_layout.addStretch()

        # Undo button
        self.undo_button = QPushButton()
        svg_data_left = QByteArray(self.svg_left)
        renderer_left = QSvgRenderer(svg_data_left)
        pixmap_left = QPixmap(30, 30)
        pixmap_left.fill(Qt.transparent)
        painter_left = QPainter(pixmap_left)
        renderer_left.render(painter_left, QRectF(0, 0, 24, 24))
        painter_left.end()
        self.undo_button.setIcon(QIcon(pixmap_left))
        self.undo_button.setIconSize(QSize(24, 24))
        self.undo_button.setFixedSize(35, 35)
        self.undo_button.setStyleSheet("""
            QPushButton {
                background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1,
                stop:0 #6366f1, stop:1 #4f46e5);
                border: none;
                padding: 0px;
                margin: 0px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1,
                stop:0 #4f46e5, stop:1 #4338ca);
            }
            QPushButton:pressed {
                background-color: #3730a3;
            }
        """)
        self.undo_button.setCursor(Qt.PointingHandCursor)
        self.undo_button.setToolTip("Undo")
        header_layout.addWidget(self.undo_button)

        # Redo button
        self.redo_button = QPushButton()
        svg_data_right = QByteArray(self.svg_right)
        renderer_right = QSvgRenderer(svg_data_right)
        pixmap_right = QPixmap(30, 30)
        pixmap_right.fill(Qt.transparent)
        painter_right = QPainter(pixmap_right)
        renderer_right.render(painter_right, QRectF(0, 0, 24, 24))
        painter_right.end()
        self.redo_button.setIcon(QIcon(pixmap_right))
        self.redo_button.setIconSize(QSize(24, 24))
        self.redo_button.setFixedSize(35, 35)
        self.redo_button.setStyleSheet(self.undo_button.styleSheet())
        self.redo_button.setCursor(Qt.PointingHandCursor)
        self.redo_button.setToolTip("Redo")
        header_layout.addWidget(self.redo_button)

        # Add header container to line layout
        line_layout.addWidget(self.header_container)

        # Create line_content_widget (the collapsible part)
        self.line_content_widget = QWidget()
        self.line_content_layout = QVBoxLayout(self.line_content_widget)
        self.line_content_layout.setContentsMargins(0, 5, 0, 0)
        self.line_content_layout.setSpacing(5)

        # Add line_content_widget to line layout
        line_layout.addWidget(self.line_content_widget)

        # Function to create checkbox rows
        def create_line_checkbox_with_pencil(checkbox_text, info_text, item_id, color_style=""):
            container = QWidget()
            container.setFixedWidth(250)
            container.setStyleSheet("""
                QWidget {
                    background-color: transparent;
                    border: none;
                    margin: 1px;
                }
            """)
            layout = QHBoxLayout(container)
            layout.setContentsMargins(0, 5, 5, 5)
            layout.setSpacing(5)
            
            checkbox = QCheckBox()
            checkbox.setFixedSize(35, 35)
            checkbox.setText("")
            checkbox.setObjectName(item_id)
            
            pencil_button = QPushButton()
            svg_data = QByteArray()
            svg_data.append(self.PENCIL_SVG)
            renderer = QSvgRenderer(svg_data)
            pixmap = QPixmap(24, 24)
            pixmap.fill(Qt.transparent)
            painter = QPainter(pixmap)
            renderer.render(painter, QRectF(pixmap.rect()))
            painter.end()
            icon = QIcon(pixmap)
            pencil_button.setIcon(icon)
            pencil_button.setIconSize(QSize(24, 24))
            pencil_button.setFixedSize(30, 30)
            pencil_button.setStyleSheet("""
                QPushButton {
                    background-color: #28a745;
                    border: none;
                    padding: 0px;
                    margin: 0px;
                    border-radius: 3px;
                }
                QPushButton:hover {
                    background-color: #218838;
                }
                QPushButton:pressed {
                    background-color: #1e7e34;
                }
            """)
            pencil_button.setCursor(Qt.PointingHandCursor)
            
            text_label = QLabel(checkbox_text)
            text_label.setStyleSheet(f"""
                QLabel {{
                    background-color: transparent;
                    border: none;
                    padding: 0px;
                    font-weight: bold;
                    font-size: 16px;
                    color: #000000;
                    text-align: left;
                }}
            """)
            
            layout.addWidget(checkbox)
            layout.addWidget(text_label, 1)
            layout.addWidget(pencil_button)
            
            return container, checkbox, text_label, pencil_button

        # # Create checkbox rows
        # Zero Line
        self.zero_container, self.zero_line, zero_label, self.zero_pencil = create_line_checkbox_with_pencil(
            "Zero Line",
            "Shows the zero reference line for elevation measurements",
            'zero_line'
        )
        self.zero_line.setStyleSheet("""
            QCheckBox {
                color: black;
                font-size: 14px;
                font-weight: bold;
            }
        """)
        self.zero_container.setVisible(False)

        # Connect state change → change label text color to purple when checked
        self.zero_line.stateChanged.connect(lambda state: zero_label.setStyleSheet("""
            QLabel {
                background-color: transparent;
                border: none;
                padding: 0px;
                font-weight: bold;
                font-size: 16px;
                color: purple;
            }
        """ if state == Qt.Checked else """
            QLabel {
                background-color: transparent;
                border: none;
                padding: 0px;
                font-weight: bold;
                font-size: 16px;
                color: #000000;
            }
        """))
        # self.zero_line.stateChanged.connect(lambda state: self.on_checkbox_changed(state, 'zero'))
        # self.zero_pencil.clicked.connect(self.edit_zero_line)
        line_layout.addWidget(self.zero_container)

        # self.zero_container.setVisible(False)
 

        # Surface Line
        self.surface_container, self.surface_baseline, surface_label, surface_pencil = create_line_checkbox_with_pencil(
            "Surface Line",
            "Shows the ground surface baseline",
            'surface_line'
        )
        self.surface_baseline.setStyleSheet("""
            QCheckBox {
                color: black;
                font-size: 14px;
                font-weight: bold;
            }
        """)
        self.surface_container.setVisible(False)

        self.surface_baseline.stateChanged.connect(lambda state: surface_label.setStyleSheet("""
            QLabel {
                background-color: transparent;
                border: none;
                padding: 0px;
                font-weight: bold;
                font-size: 16px;
                color: green;
            }
        """ if state == Qt.Checked else """
            QLabel {
                background-color: transparent;
                border: none;
                padding: 0px;
                font-weight: bold;
                font-size: 16px;
                color: #000000;
            }
        """))

        line_layout.addWidget(self.surface_container)

        # Construction Line
        self.construction_container, self.construction_line, construction_label, construction_pencil = create_line_checkbox_with_pencil(
            "Construction Line",
            "Shows the construction reference line",
            'construction_line'
        )
        self.construction_line.setStyleSheet("""
            QCheckBox {
                color: black;
                font-size: 14px;
                font-weight: bold;
            }
        """)
        self.construction_container.setVisible(False)

        self.construction_line.stateChanged.connect(lambda state: construction_label.setStyleSheet("""
            QLabel {
                background-color: transparent;
                border: none;
                padding: 0px;
                font-weight: bold;
                font-size: 16px;
                color: red;
            }
        """ if state == Qt.Checked else """
            QLabel {
                background-color: transparent;
                border: none;
                padding: 0px;
                font-weight: bold;
                font-size: 16px;
                color: #000000;
            }
        """))

        line_layout.addWidget(self.construction_container)

        # Road Surface Line
        self.road_surface_container, self.road_surface_line, road_surface_label, road_pencil = create_line_checkbox_with_pencil(
            "Road Surface Line",
            "Shows the road surface elevation profile",
            'road_surface_line'
        )
        self.road_surface_line.setStyleSheet("""
            QCheckBox {
                color: black;
                font-size: 14px;
                font-weight: bold;
            }
        """)
        self.road_surface_container.setVisible(False)

        self.road_surface_line.stateChanged.connect(lambda state: road_surface_label.setStyleSheet("""
            QLabel {
                background-color: transparent;
                border: none;
                padding: 0px;
                font-weight: bold;
                font-size: 16px;
                color: blue;
            }
        """ if state == Qt.Checked else """
            QLabel {
                background-color: transparent;
                border: none;
                padding: 0px;
                font-weight: bold;
                font-size: 16px;
                color: #000000;
            }
        """))

        line_layout.addWidget(self.road_surface_container)

        # Bridge-specific Zero Line
        self.bridge_zero_container, self.bridge_zero_line, bridge_zero_label, self.bridge_zero_pencil = create_line_checkbox_with_pencil(
            "Zero Line",
            "Shows the bridge zero reference line for elevation measurements",
            'zero_line'
        )
        self.bridge_zero_line.setStyleSheet("""
            QCheckBox {
                color: black;
                font-size: 14px;
                font-weight: bold;
            }
        """)
        self.bridge_zero_container.setVisible(False)

        self.bridge_zero_line.stateChanged.connect(lambda state: bridge_zero_label.setStyleSheet("""
            QLabel {
                background-color: transparent;
                border: none;
                padding: 0px;
                font-weight: bold;
                font-size: 16px;
                color: purple;
            }
        """ if state == Qt.Checked else """
            QLabel {
                background-color: transparent;
                border: none;
                padding: 0px;
                font-weight: bold;
                font-size: 16px;
                color: #000000;
            }
        """))

        line_layout.addWidget(self.bridge_zero_container)

        # Projection Line
        self.projection_container, self.projection_line, projection_label, projection_pencil = create_line_checkbox_with_pencil(
            " Projection Line",
            "Shows the projection elevation profile",
            'projection_line'
        )
        self.projection_line.setStyleSheet("""
            QCheckBox {
                color: black;
                font-size: 14px;
                font-weight: bold;
            }
        """)
        self.projection_container.setVisible(False)

        self.projection_line.stateChanged.connect(lambda state: projection_label.setStyleSheet("""
            QLabel {
                background-color: transparent;
                border: none;
                padding: 0px;
                font-weight: bold;
                font-size: 16px;
                color: green;
            }
        """ if state == Qt.Checked else """
            QLabel {
                background-color: transparent;
                border: none;
                padding: 0px;
                font-weight: bold;
                font-size: 16px;
                color: #000000;
            }
        """))

        line_layout.addWidget(self.projection_container)

        # Construction Dots Line
        self.construction_dots_container, self.construction_dots_line, construction_dots_label, self.construction_dots_pencil = create_line_checkbox_with_pencil(
            "Construction Dots",
            "Shows the construction reference dots line",
            'construction_dots_line'
        )
        self.construction_dots_line.setStyleSheet("""
            QCheckBox {
                color: black;
                font-size: 14px;
                font-weight: bold;
            }
        """)
        self.construction_dots_container.setVisible(False)

        self.construction_dots_line.stateChanged.connect(lambda state: construction_dots_label.setStyleSheet("""
            QLabel {
                background-color: transparent;
                border: none;
                padding: 0px;
                font-weight: bold;
                font-size: 16px;
                color: red;
            }
        """ if state == Qt.Checked else """
            QLabel {
                background-color: transparent;
                border: none;
                padding: 0px;
                font-weight: bold;
                font-size: 16px;
                color: #000000;
            }
        """))

        line_layout.addWidget(self.construction_dots_container)

        # Deck Line
        self.deck_line_container, self.deck_line, deck_label, self.deck_pencil = create_line_checkbox_with_pencil(
            "Deck Line",
            "Shows the deck elevation profile",
            'deck_line'
        )
        self.deck_line.setStyleSheet("""
            QCheckBox {
                color: black;
                font-size: 14px;
                font-weight: bold;
            }
        """)
        self.deck_line_container.setVisible(False)

        self.deck_line.stateChanged.connect(lambda state: deck_label.setStyleSheet("""
            QLabel {
                background-color: transparent;
                border: none;
                padding: 0px;
                font-weight: bold;
                font-size: 16px;
                color: blue;
            }
        """ if state == Qt.Checked else """
            QLabel {
                background-color: transparent;
                border: none;
                padding: 0px;
                font-weight: bold;
                font-size: 16px;
                color: #000000;
            }
        """))

        line_layout.addWidget(self.deck_line_container)

# ================================================================================================================================== 

        # Additional buttons - "Add Material Line" (now always visible)
        self.add_material_line_button = QPushButton("Add Material Line")
        self.add_material_line_button.setStyleSheet("""
            QPushButton {
                background-color: #28a745;  /* Green */
                color: white;
                border: none;
                padding: 10px;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #218838;
            }
            QPushButton:pressed {
                background-color: #1e7e34;
            }
        """)
        # Button is now always visible from the start
        self.add_material_line_button.setVisible(True)

        line_layout.addWidget(self.add_material_line_button)

        # Create a dedicated layout for material items (to control order)
        self.material_items_layout = QVBoxLayout()
        self.material_items_layout.setSpacing(6)
        self.material_items_layout.setContentsMargins(0, 10, 0, 0)
        self.material_items_layout.setAlignment(Qt.AlignTop)

        # Add this layout right after the button
        line_layout.addLayout(self.material_items_layout)

        line_layout.addStretch(1)

        # Storage for material configurations and UI items
        self.material_configs = []   # list of dicts with material data
        self.material_items = []     # list of UI item dicts

        # Additional buttons
        self.preview_button = QPushButton("Curve")
        self.elivation_angle_button = QPushButton("Elevation Angle")

        # Angle buttons container
        self.angle_buttons_container = QWidget()
        self.angle_buttons_container.setFixedHeight(40)
        self.angle_buttons_layout = QHBoxLayout(self.angle_buttons_container)
        self.angle_buttons_layout.setContentsMargins(0, 0, 0, 0)
        self.angle_buttons_layout.setSpacing(5)
        
        # Set minimum heights for consistency
        self.preview_button.setFixedHeight(35)
        self.preview_button.setFixedWidth(80)
        
        self.elivation_angle_button.setFixedHeight(35)
        self.elivation_angle_button.setFixedWidth(160)

        self.angle_buttons_layout.addWidget(self.preview_button)
        self.angle_buttons_layout.addWidget(self.elivation_angle_button)
        
        self.angle_buttons_layout.addWidget(self.preview_button)
        self.angle_buttons_layout.addWidget(self.elivation_angle_button)

        self.threed_map_button = QPushButton("Map on 3D")
        self.save_button = QPushButton("Save")
        
        self.preview_button.setStyleSheet("""
            QPushButton {
                background-color: #808080;
                color: white;
                border: none;
                padding: 0px;
                border-radius: 5px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #6E6E6E; }
            QPushButton:pressed { background-color: #5A5A5A; }
        """)

        self.elivation_angle_button.setStyleSheet("""
            QPushButton {
                background-color: #808080;
                color: white;
                border: none;
                padding: 0px;
                border-radius: 5px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #6E6E6E; }
            QPushButton:pressed { background-color: #5A5A5A; }
        """)
        
        self.threed_map_button.setStyleSheet("""
            QPushButton {
                background-color: #008CBA;
                color: white;
                border: none;
                padding: 10px;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #007bb5; }
            QPushButton:pressed { background-color: #006f9a; }
        """)
        
        self.save_button.setStyleSheet("""
            QPushButton {
                background-color: #FF9800;
                color: white;
                border: none;
                padding: 10px;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #F57C00; }
            QPushButton:pressed { background-color: #EF6C00; }
        """)
        
        self.preview_button.setVisible(False)
        self.elivation_angle_button.setVisible(False)
        self.threed_map_button.setVisible(False)
        self.save_button.setVisible(False)
        
        line_layout.addWidget(self.angle_buttons_container)
        line_layout.addWidget(self.threed_map_button)
        line_layout.addWidget(self.save_button)

        # Graph Canvas
        self.figure = Figure(dpi=100)
        base_width = max(self.total_distance / 3.0, 10)  # Minimum 10 inches
        self.figure.set_size_inches(base_width, 6)
        self.canvas = FigureCanvas(self.figure)

        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.canvas.setMinimumWidth(800)  # Adjust as needed

        self.ax = self.figure.add_subplot(111)
        self.ax.grid(True, which='both', linestyle='-', linewidth=0.5, alpha=0.7)
        self.ax.set_xlim(0, self.total_distance)
        self.ax.set_ylim(-3, 3)

        # Set axis labels and title with proper padding
        self.ax.set_xlabel('Y (Distance)', fontsize=12, labelpad=10)  # Added labelpad
        self.ax.set_ylabel('Z (Elevation)', fontsize=12, labelpad=10)  # Added labelpad
        self.ax.set_title('Road Construction Layers', fontsize=14, fontweight='bold', color='#4A148C', pad=15)  # Added pad

        self.ax.xaxis.set_major_locator(ticker.MultipleLocator(5))
        self.ax.yaxis.set_major_locator(ticker.MultipleLocator(1))

        # Adjust subplot parameters to make room for labels
        self.figure.subplots_adjust(left=0.15, bottom=0.15, right=0.95, top=0.90)

        self.annotation = self.ax.annotate("", xy=(0,0), xytext=(20,20), textcoords="offset points",
                                        bbox=dict(boxstyle="round,pad=0.5"), arrowprops=dict(arrowstyle="->"),
                                        ha='left', va='bottom')
        self.annotation.set_visible(False)

        # REMOVE or adjust the tight_layout call
        # self.figure.tight_layout()  # Comment out or remove this line
        # Or use it with padding:
        # self.figure.tight_layout(pad=2.0)  # If you prefer tight_layout with padding

        self.canvas.draw()

        # ---------------------------------------------------------------------
        # Zoom Controls Toolbar - UPDATED WITH TOGGLE START/STOP BUTTON
        # ---------------------------------------------------------------------
        zoom_toolbar = QWidget()
        zoom_toolbar.setFixedHeight(40)
        zoom_toolbar.setStyleSheet("""
            QWidget {
                background-color: #F5F5F5;
                border: 1px solid #CCCCCC;
                border-radius: 5px;
                margin: 0px;
                padding: 0px;
            }
        """)

        zoom_layout = QHBoxLayout(zoom_toolbar)
        zoom_layout.setContentsMargins(8, 2, 8, 2)  # Reduced margins
        zoom_layout.setSpacing(5)  # Reduced spacing between controls

        # Zoom label
        zoom_label = QLabel("Zoom:")
        zoom_label.setStyleSheet("font-weight: bold; color: #333;")
        zoom_layout.addWidget(zoom_label)

        # Zoom out button
        self.zoom_out_button = QPushButton("-")
        self.zoom_out_button.setFixedWidth(30)
        self.zoom_out_button.setFixedHeight(30)
        self.zoom_out_button.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                border-radius: 3px;
                font-weight: bold;
                font-size: 16px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:pressed {
                background-color: #0D47A1;
            }
        """)
        zoom_layout.addWidget(self.zoom_out_button)

        # Zoom level display
        self.zoom_label = QLabel("100%")
        self.zoom_label.setFixedWidth(60)
        self.zoom_label.setAlignment(Qt.AlignCenter)
        self.zoom_label.setStyleSheet("""
            QLabel {
                background-color: white;
                border: 1px solid #CCCCCC;
                border-radius: 3px;
                padding: 5px;
                font-weight: bold;
            }
        """)
        zoom_layout.addWidget(self.zoom_label)

        # Zoom in button
        self.zoom_in_button = QPushButton("+")
        self.zoom_in_button.setFixedWidth(30)
        self.zoom_in_button.setFixedHeight(30)
        self.zoom_in_button.setStyleSheet(self.zoom_out_button.styleSheet())
        zoom_layout.addWidget(self.zoom_in_button)

        # Reset zoom button
        self.reset_zoom_button = QPushButton("Reset")
        self.reset_zoom_button.setFixedHeight(30)
        self.reset_zoom_button.setStyleSheet("""
            QPushButton {
                background-color: #FF9800;
                color: white;
                border: none;
                border-radius: 3px;
                font-weight: bold;
                padding: 0 15px;
            }
            QPushButton:hover {
                background-color: #F57C00;
            }
        """)
        zoom_layout.addWidget(self.reset_zoom_button)

        # Simple slider
        zoom_layout.addWidget(QLabel("Scale:"))
        self.zoom_slider = QSlider(Qt.Horizontal)
        self.zoom_slider.setRange(10, 200)  # 10% to 200%
        self.zoom_slider.setValue(100)
        self.zoom_slider.setTickPosition(QSlider.NoTicks)
        self.zoom_slider.setFixedWidth(100)
        zoom_layout.addWidget(self.zoom_slider)

        # Pan controls
        zoom_layout.addWidget(QLabel("Pan:"))
        self.pan_left_button = QPushButton("◀")
        self.pan_left_button.setFixedWidth(30)
        self.pan_left_button.setFixedHeight(30)
        self.pan_left_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                border-radius: 3px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #388E3C; }
        """)
        zoom_layout.addWidget(self.pan_left_button)

        self.pan_right_button = QPushButton("▶")
        self.pan_right_button.setFixedWidth(30)
        self.pan_right_button.setFixedHeight(30)
        self.pan_right_button.setStyleSheet(self.pan_left_button.styleSheet())
        zoom_layout.addWidget(self.pan_right_button)

        self.pan_up_button = QPushButton("▲")
        self.pan_up_button.setFixedWidth(30)
        self.pan_up_button.setFixedHeight(30)
        self.pan_up_button.setStyleSheet(self.pan_left_button.styleSheet())
        zoom_layout.addWidget(self.pan_up_button)

        self.pan_down_button = QPushButton("▼")
        self.pan_down_button.setFixedWidth(30)
        self.pan_down_button.setFixedHeight(30)
        self.pan_down_button.setStyleSheet(self.pan_left_button.styleSheet())
        zoom_layout.addWidget(self.pan_down_button)

        # Auto-fit button
        self.autofit_button = QPushButton("Auto Fit")
        self.autofit_button.setFixedHeight(30)
        self.autofit_button.setStyleSheet("""
            QPushButton {
                background-color: #9C27B0;
                color: white;
                border: none;
                border-radius: 3px;
                font-weight: bold;
                padding: 0 15px;
            }
            QPushButton:hover { background-color: #7B1FA2; }
        """)
        zoom_layout.addWidget(self.autofit_button)

        # === NEW: Toggle Start / Stop Button ===
        self.start_stop_button = QPushButton("Start")
        self.start_stop_button.setFixedHeight(30)
        self.start_stop_button.setCheckable(True)  # Allows toggle state
        self.start_stop_button.setStyleSheet("""
            QPushButton {
                color: white;
                border: none;
                border-radius: 3px;
                font-weight: bold;
                padding: 0 15px;
            }
            QPushButton:checked {
                background-color: #D32F2F;  /* Red when "Stop" */
            }
            QPushButton:!checked {
                background-color: #4CAF50;  /* Purple when "Start" */
            }
            QPushButton:hover:checked {
                background-color: #B71C1C;
            }
            QPushButton:hover:!checked {
                background-color: #4CAF50;
            }
        """)
        # Optional: connect to your actual start/stop logic here
        # self.start_stop_button.clicked.connect(self.on_start_stop_clicked)
        zoom_layout.addWidget(self.start_stop_button)

        # Connect the toggle behavior to update text
        def on_start_stop_toggled(checked):
            if checked:
                self.start_stop_button.setText("Stop")
            else:
                self.start_stop_button.setText("Start")

        self.start_stop_button.toggled.connect(on_start_stop_toggled)

        # Add stretch to push everything left
        zoom_layout.addStretch()

        # Add zoom toolbar to the layout BEFORE the graph
        bottom_layout.addWidget(zoom_toolbar)

        # Create horizontal layout for line section and canvas
        content_layout_bottom = QHBoxLayout()
        content_layout_bottom.setContentsMargins(0, 0, 0, 0)
        content_layout_bottom.setSpacing(0)
        content_layout_bottom.addWidget(self.line_section)
        content_layout_bottom.addWidget(self.canvas, 4)
        bottom_layout.addLayout(content_layout_bottom, 1)

        # ------------------------------------------------------------------
        # HERARCHY SECTION – Controls
        # ------------------------------------------------------------------
        self.herarchy_section = QFrame()
        self.herarchy_section.setVisible(False)

        # Add middle, scale, and bottom sections to right layout
        right_layout.addWidget(middle_section, 3)  # 3 parts for visualization
        right_layout.addWidget(scale_section, 1)   # 1 part for scale
        right_layout.addWidget(self.bottom_section, 1)  # 1 part for controls
        right_layout.addWidget(self.herarchy_section, 1)  # 1 part for controls

        # Add right section to content layout
        content_layout.addWidget(scroll_area, 3)

        # --------------------------------
        # Measurement SECTION – Settings
        # ------------------------------------------------------------------
        self.main_measurement_section = QFrame()  # Changed here
        self.main_measurement_section.setFrameStyle(QFrame.Box | QFrame.Raised)
        self.main_measurement_section.setStyleSheet("""
            QFrame { 
            background-color: #DCEDC8;      /* panel color */
            border: 1px solid black;      /* thin black border */
            border-radius: 6px;           /* rounded corners */
            background-color: #E8F5E9; 
            margin: 3px;
            }
        """)
        self.main_measurement_section.setMinimumWidth(300)
        self.main_measurement_section.setMaximumWidth(300)
        self.main_measurement_section.setVisible(False)

        self.main_measurement_layout = QVBoxLayout(self.main_measurement_section)
        # Remove margins and spacing to minimize empty space
        self.main_measurement_layout.setContentsMargins(5, 5, 5, 5) 
        self.main_measurement_layout.setSpacing(3)

        # Add right section to content layout
        content_layout.addWidget(self.main_measurement_section, 1)

        # Add content widget to main layout
        main_layout.addWidget(content_widget, 1)

        # Create central widget and set layout
        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        # Setup zoom and pan controls
        self.setup_zoom_controls()

        # Initial zoom display
        self.update_zoom_display()

        self.setup_scroll_behavior()

    def create_progress_bar(self):
        """Create and configure the progress bar widget"""
        self.progress_bar = QWidget()
        self.progress_bar.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        self.progress_bar.setStyleSheet("""
            background-color: rgba(50, 50, 50, 220);
            border-radius: 8px;
            border: 1px solid #444;
        """)
        self.progress_bar.setFixedSize(500, 300) # Increased width to accommodate file size
        layout = QVBoxLayout(self.progress_bar)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(20)
        # Loading label
        self.loading_label = QLabel("Loading Point Cloud Data...")
        self.loading_label.setStyleSheet("""
            QLabel {
                font-size: 18px;
                font-weight: bold;
                color: white;
            }
        """)
        self.loading_label.setAlignment(Qt.AlignCenter)
        # File info container (horizontal layout for file name and size)
        file_info_container = QWidget()
        file_info_layout = QHBoxLayout(file_info_container)
        file_info_layout.setContentsMargins(0, 0, 0, 0)
        file_info_layout.setSpacing(10)
        # File name label
        self.file_name_label = QLabel("")
        self.file_name_label.setStyleSheet("""
            QLabel {
                font-size: 18px;
                color: white;
            }
        """)
        self.file_name_label.setAlignment(Qt.AlignCenter)
        self.file_name_label.setWordWrap(True)
        # File size label
        self.file_size_label = QLabel("")
        self.file_size_label.setStyleSheet("""
            QLabel {
                font-size: 18px;
                color: white;
            }
        """)
        self.file_size_label.setAlignment(Qt.AlignCenter)
        file_info_layout.addWidget(self.file_name_label, 70) # 70% width
        file_info_layout.addWidget(self.file_size_label, 30) # 30% width
        # Progress bar
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.progress.setTextVisible(False)
        self.progress.setFixedHeight(15)
        self.progress.setStyleSheet("""
            QProgressBar {
                border: 1px solid #444;
                border-radius: 6px;
                background-color: #333;
            }
            QProgressBar::chunk {
                background-color: #4CAF50;
                border-radius: 5px;
            }
        """)
        # Percentage label
        self.percentage_label = QLabel("0%")
        self.percentage_label.setStyleSheet("""
            QLabel {
                font-size: 16px;
                font-weight: bold;
                color: white;
            }
        """)
        self.percentage_label.setAlignment(Qt.AlignCenter)
        # Add widgets to layout
        layout.addWidget(self.loading_label)
        layout.addWidget(file_info_container)
        layout.addWidget(self.progress)
        layout.addWidget(self.percentage_label)
        # Center the progress bar on screen but shifted slightly to the right
        screen_geometry = QApplication.desktop().screenGeometry()
        x = (screen_geometry.width() - self.progress_bar.width()) // 2 + 150 # Shift 100 pixels right
        y = (screen_geometry.height() - self.progress_bar.height()) // 2
        self.progress_bar.move(x, y)

    def show_progress_bar(self, file_path=None):
        """Show and position the progress bar"""
        if file_path:
            file_name = os.path.basename(file_path)
            self.file_name_label.setText(f"File: {file_name}")
            # Get and format file size
            try:
                file_size_bytes = os.path.getsize(file_path)
                if file_size_bytes < 1024 * 1024: # Less than 1 MB
                    size_str = f"{file_size_bytes/1024:.1f} KB"
                elif file_size_bytes < 1024 * 1024 * 1024: # Less than 1 GB
                    size_str = f"{file_size_bytes/(1024*1024):.1f} MB"
                else: # GB or more
                    size_str = f"{file_size_bytes/(1024*1024*1024):.1f} GB"
                self.file_size_label.setText(f"Size: {size_str}")
            except:
                self.file_size_label.setText("Size: Unknown")
        self.progress.setValue(0)
        self.percentage_label.setText("0%")
        self.progress_bar.show()
        QApplication.processEvents() # Force UI update

    def update_progress(self, value, message=None):
        """Update progress bar value and optionally the message"""
        self.progress.setValue(value)
        self.percentage_label.setText(f"{value}%")
        if message:
            self.loading_label.setText(message)
        QApplication.processEvents() # Ensure UI updates

    def hide_progress_bar(self):
        """Hide the progress bar with a smooth fade-out"""
        self.progress_bar.hide()
        self.progress.setValue(0)
        self.percentage_label.setText("0%")

    # def toggle_message_section(self):
    #     self.message_visible = not self.message_visible
    #     self.message_section.setVisible(self.message_visible)
    #     self.message_button.setText("Hide Message" if self.message_visible else "Message")
    
    # def toggle_worksheet_options(self):
    #     checked = self.worksheet_button.isChecked()
    #     self.sub_buttons_widget.setVisible(checked)
    #     self.worksheet_button.setText("Worksheet ▼" if checked else "Worksheet")

    # def toggle_design_options(self):
    #     checked = self.design_button.isChecked()
    #     self.sub_design_buttons_widget.setVisible(checked)
    #     self.design_button.setText("Design ▼" if checked else "Design")
    
    # def toggle_construction_options(self):
    #     checked = self.construction_button.isChecked()
    #     self.sub_construction_buttons_widget.setVisible(checked)
    #     self.construction_button.setText("Construction ▼" if checked else "Construction")

    # def toggle_measurement_options(self):
    #     checked = self.measurement_button.isChecked()
    #     self.sub_measurement_buttons_widget.setVisible(checked)
    #     self.measurement_button.setText("Measurement ▼" if checked else "Measurement")

    def toggle_line_section(self):
        """Toggle visibility of the line section"""
        if self.line_content_widget.isVisible():
            # Collapse the section
            self.line_content_widget.hide()
            self.collapse_button.setText("▶")
            self.collapse_button.setToolTip("Open Line Section")
            # Hide undo/redo buttons
            self.undo_button.hide()
            self.redo_button.hide()
            # Set fixed width for collapsed state
            self.line_section.setFixedWidth(80)
            # Update the layout to remove space for hidden buttons
            self.header_container.layout().setContentsMargins(0, 5, 0, 5)
        else:
            # Expand the section
            self.line_content_widget.show()
            self.collapse_button.setText("◀")
            self.collapse_button.setToolTip("Close Line Section")
            # Show undo/redo buttons
            self.undo_button.show()
            self.redo_button.show()
            self.line_section.setFixedWidth(270)

        # Force layout update
        self.header_container.updateGeometry()
        self.line_section.updateGeometry()
        QApplication.processEvents()  # Force immediate UI update
        self.canvas.draw()

    def setup_zoom_controls(self):
        """Connect zoom controls to their functions"""
        self.zoom_in_button.clicked.connect(self.zoom_in_simple)
        self.zoom_out_button.clicked.connect(self.zoom_out_simple)
        self.reset_zoom_button.clicked.connect(self.reset_zoom_simple)
        self.zoom_slider.valueChanged.connect(self.zoom_slider_changed_simple)
        
        # Pan controls
        self.pan_left_button.clicked.connect(self.pan_left_simple)
        self.pan_right_button.clicked.connect(self.pan_right_simple)
        self.pan_up_button.clicked.connect(self.pan_up_simple)
        self.pan_down_button.clicked.connect(self.pan_down_simple)
        
        # Set up mouse wheel zoom
        self.canvas.mpl_connect('scroll_event', self.on_mouse_wheel_simple)
        
        # Initialize zoom state
        self.current_zoom = 100  # 100%
        self.original_xlim = (0, self.total_distance)
        self.original_ylim = (-3, 3)

        self.autofit_button.clicked.connect(self.autofit_graph_simple)

    def zoom_in_simple(self):
        """Zoom in by 20%"""
        self.current_zoom = min(500, self.current_zoom * 1.2)  # Cap at 500%
        self.apply_zoom()
        self.update_zoom_display()

    def zoom_out_simple(self):
        """Zoom out by 20%"""
        self.current_zoom = max(10, self.current_zoom / 1.2)  # Minimum 10%
        self.apply_zoom()
        self.update_zoom_display()

    def reset_zoom_simple(self):
        """Reset zoom to original view"""
        self.current_zoom = 100
        self.ax.set_xlim(self.original_xlim)
        self.ax.set_ylim(self.original_ylim)
        self.zoom_slider.setValue(100)
        self.update_zoom_display()
        self.canvas.draw()

    def zoom_slider_changed_simple(self, value):
        """Handle zoom slider changes"""
        self.current_zoom = value
        self.apply_zoom()
        self.update_zoom_display()

    def apply_zoom(self):
        """Apply the current zoom level to the graph - Zoom relative to visible left"""
        current_xlim = self.ax.get_xlim()
        current_left = current_xlim[0]  # Get current left position
        
        # Calculate new range based on zoom level
        original_range = self.original_xlim[1] - self.original_xlim[0]
        new_range = original_range * (100 / self.current_zoom)
        
        # Keep the current left position fixed, adjust right based on zoom
        new_xlim = (current_left, current_left + new_range)
        
        # Apply new limits
        self.ax.set_xlim(new_xlim)
        
        # Keep y-axis at original scale for now
        self.ax.set_ylim(self.original_ylim)
        
        self.canvas.draw()

    def update_zoom_display(self):
        """Update zoom label"""
        self.zoom_label.setText(f"{int(self.current_zoom)}%")

    def on_mouse_wheel_simple(self, event):
        """Mouse wheel zoom - simpler version"""
        if event.inaxes != self.ax:
            return
        
        # Try to detect Ctrl - check both string and Qt modifiers
        ctrl_pressed = False
        
        # Check event.key (matplotlib's representation)
        if event.key and isinstance(event.key, str) and 'control' in event.key.lower():
            ctrl_pressed = True
        
        # Check Qt modifiers if available
        if not ctrl_pressed and hasattr(event, 'guiEvent') and event.guiEvent:
            from PyQt5.QtCore import Qt
            if event.guiEvent.modifiers() & Qt.ControlModifier:
                ctrl_pressed = True
        
        # If Ctrl is pressed, zoom
        if ctrl_pressed:
            if event.button == 'up':
                self.zoom_in_simple()
            elif event.button == 'down':
                self.zoom_out_simple()

    def pan_left_simple(self):
        """Pan graph to the left, but not beyond 0"""
        current_xlim = self.ax.get_xlim()
        
        # Calculate pan amount (10% of current width)
        pan_amount = (current_xlim[1] - current_xlim[0]) * 0.1
        
        # Calculate new left position
        new_left = current_xlim[0] - pan_amount
        
        # Don't go below 0
        if new_left < 0:
            new_left = 0
        
        # Calculate new right position
        new_right = current_xlim[1] - (current_xlim[0] - new_left)
        
        self.ax.set_xlim(new_left, new_right)
        self.canvas.draw()

    def pan_right_simple(self):
        """Pan graph to the right"""
        current_xlim = self.ax.get_xlim()
        pan_amount = (current_xlim[1] - current_xlim[0]) * 0.1
        self.ax.set_xlim(current_xlim[0] + pan_amount, current_xlim[1] + pan_amount)
        self.canvas.draw()

    def pan_up_simple(self):
        """Pan graph up"""
        current_ylim = self.ax.get_ylim()
        pan_amount = (current_ylim[1] - current_ylim[0]) * 0.1
        self.ax.set_ylim(current_ylim[0] + pan_amount, current_ylim[1] + pan_amount)
        self.canvas.draw()

    def pan_down_simple(self):
        """Pan graph down"""
        current_ylim = self.ax.get_ylim()
        pan_amount = (current_ylim[1] - current_ylim[0]) * 0.1
        self.ax.set_ylim(current_ylim[0] - pan_amount, current_ylim[1] - pan_amount)
        self.canvas.draw()

    def autofit_graph_simple(self):
        """Auto-fit the graph to show all lines"""
        try:
            # Get all line data
            all_x = []
            all_y = []
            
            # Check each line type for data
            for line_type, data in self.line_types.items():
                for polyline in data['polylines']:
                    if polyline and len(polyline) > 0:
                        xs = [p[0] for p in polyline]
                        ys = [p[1] for p in polyline]
                        all_x.extend(xs)
                        all_y.extend(ys)
            
            if all_x and all_y:
                # Calculate bounds with 10% padding
                x_min, x_max = min(all_x), max(all_x)
                y_min, y_max = min(all_y), max(all_y)
                
                x_padding = (x_max - x_min) * 0.1
                y_padding = (y_max - y_min) * 0.1
                
                # Apply new bounds
                self.ax.set_xlim(x_min - x_padding, x_max + x_padding)
                self.ax.set_ylim(y_min - y_padding, y_max + y_padding)
                
                # Update zoom display
                visible_range = (x_max + x_padding) - (x_min - x_padding)
                self.current_zoom = (self.original_xlim[1] / visible_range) * 100
                self.update_zoom_display()
                self.zoom_slider.setValue(int(self.current_zoom))
                
                self.canvas.draw()
                self.message_text.append("Graph auto-fitted to show all data.")
            else:
                self.message_text.append("No data available for auto-fit.")
        except Exception as e:
            self.message_text.append(f"Auto-fit error: {str(e)}")

# =============================================================================
    def add_layer_to_panel(self, layer_name: str, dimension: str):
        """
        Adds a layer label to the correct panel (3D or 2D Layers).
        Used for both worksheet initial layers and design layers.
        """
        label = QLabel(f"• {layer_name}")
        label.setStyleSheet("""
            QLabel {
                padding: 8px 12px;
                background-color: rgba(255, 255, 255, 0.9);
                border-radius: 8px;
                margin: 3px 8px;
                font-size: 13px;
                color: #0D47A1;
                border-left: 4px solid #1976D2;
            }
            QLabel:hover {
                background-color: #BBDEFB;
            }
        """)
        label.setToolTip(f"{dimension} Layer: {layer_name}")

        if dimension == "3D":
            if self.three_D_layers_layout:
                self.three_D_layers_layout.insertWidget(self.three_D_layers_layout.count() - 1, label)
        elif dimension == "2D":
            if self.two_D_layers_layout:
                self.two_D_layers_layout.insertWidget(self.two_D_layers_layout.count() - 1, label)


 # ==================================================================================================================================
# Define function for the Load Point Cloud Data File & Start Measurements Buttons:
    def create_file_load_section(self):
        self.file_load_group = QGroupBox("File Load") # Section Name
        self.file_load_layout = QVBoxLayout()

        # Remove margins and spacing to minimize empty space
        self.file_load_layout.setContentsMargins(5, 5, 5, 5) 
        self.file_load_layout.setSpacing(3)
        
        self.start_button = QPushButton("Start Measurement") # Button for the start Measurements
        
        # self.start_button.setEnabled(False)
        
        self.file_load_layout.addWidget(self.start_button)

        self.file_load_group.setLayout(self.file_load_layout)

        self.main_measurement_layout.addWidget(self.file_load_group)

        self.file_load_group.setStyleSheet("""
            QGroupBox {
            border: 1px solid gray;
            border-radius: 5px;
            margin-top: 0.5em;
            }
            QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 3px;
            }
            """)
        
# ==================================================================================================================================
# Define function for Measurement Section:
    def create_measurement_section(self):
        self.measurement_group = QGroupBox("Measurement") # Section Name
        self.measurement_layout = QVBoxLayout()

        self.measurement_layout.setContentsMargins(5, 5, 5, 5)
        
        # Add metrics selection dropdown at the top
        self.metrics_group = QGroupBox("Measurement Metrics")
        metrics_layout = QHBoxLayout()
        self.metrics_group.setFixedHeight(70)
        self.metrics_group.setFixedWidth(230)
        
        self.metrics_combo = QComboBox()
        self.metrics_combo.addItems(["Meter", "Centimeter", "Millimeter"])
        self.metrics_combo.currentTextChanged.connect(self.update_measurement_metrics)
        
        metrics_layout.addWidget(QLabel("Units:"))
        metrics_layout.addWidget(self.metrics_combo)
        self.metrics_group.setLayout(metrics_layout)
        self.measurement_layout.addWidget(self.metrics_group)

        # Create a container widget for measurement buttons that will be shown/hidden
        self.measurement_buttons_container = QWidget()
        self.measurement_buttons_layout = QVBoxLayout()
        self.measurement_buttons_container.setLayout(self.measurement_buttons_layout)
        
        # Measurement buttons
        self.line_button = QPushButton("Line")  # New Line button
        self.line_menu = QMenu()  # Create dropdown menu
        self.vertical_line_action = QAction("Vertical Line", self)  # Vertical line option
        self.horizontal_line_action = QAction("Horizontal Line", self)  # Horizontal line option
        self.measurement_line_action = QAction("Measurement Line", self)
        self.presized_button = QPushButton("Presized")
        self.presized_button.setVisible(False)  # Start hidde
        self.line_menu.addAction(self.vertical_line_action)
        self.line_menu.addAction(self.horizontal_line_action)
        self.line_menu.addAction(self.measurement_line_action)
        self.line_button.setMenu(self.line_menu)  # Attach menu to button

        self.round_pillar_polygon_button = QPushButton("Round Pillar Polygon")
        self.polygon_button = QPushButton("Polygon") # Polygon button to measure a surface area of uneven structure land, angles, and Distances
        
        # Add Complete Polygon button (initially hidden
        self.complete_polygon_button = QPushButton("Complete Polygon")
        self.complete_polygon_button.setVisible(False)  # Start hidden
        self.pillar_dimension = QPushButton("Pillar Dimensions") 

        self.cut_hill_button = QPushButton("Cut Hill")
        self.excavate_button = QPushButton("Excavate and Remove")
        self.excavate_button.setStyleSheet("""
            QPushButton {
                background-color: #DC3545;  /* Red color for destructive action */
                color: white;
                border: none;
                padding: 10px;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #C82333;  /* Darker red on hover */
            }
            QPushButton:pressed {
                background-color: #BD2130;  /* Even darker when pressed */
            }
        """)
        self.excavate_button.setVisible(False)

        self.Ohe_angle_button = QPushButton("OHE Pole Angle with Rail")
        self.defect_angle_button = QPushButton("Angle of Defect")        
        self.all_angle_button = QPushButton("Angle")        

        self.elivation_btn_catenary_raillevel_button = QPushButton("Distance between Catenary Wire to Rail")
        self.elivation_btn_contact_raillevel_button = QPushButton("Distance between Contact Wire to Rail")

        self.elivation_btn_catenary_contact_button = QPushButton("Distance between Contact and Catenary")
        
        self.elivation_btn_catenary_contact_menu = QMenu()  # Create dropdown menu
        self.contact_action = QAction("Contact Points", self) 
        self.catenary_action = QAction("Catenary Points", self)  
        self.elivation_btn_catenary_contact_menu.addAction(self.contact_action)
        self.elivation_btn_catenary_contact_menu.addAction(self.catenary_action)
        self.elivation_btn_catenary_contact_button.setMenu(self.elivation_btn_catenary_contact_menu)
        
        # Add Complete Eclipse button (initially hidden)
        self.complete_curve_button = QPushButton("Complete Curve")
        self.complete_curve_button.setVisible(False) 

        self.baseline_button = QPushButton("Baseline")
        self.inclination_button = QPushButton("Inclination Angle")

        crop_button_row = QHBoxLayout()
        self.crop_button = QPushButton("Crop Selected Area")
        crop_button_row.addWidget(self.crop_button)
        
        self.save_crop_button = QPushButton("Save Cropped Data")
        self.save_crop_button.setEnabled(False)
        crop_button_row.addWidget(self.save_crop_button)
        self.measurement_buttons_layout.addLayout(crop_button_row)

        # Checkboxes
        self.height_check = QCheckBox("Height") # Checkbox for the height measurement
        self.height_input = QLineEdit()
        self.height_input.setPlaceholderText("Enter the height in meters")
        self.height_input.setValidator(QDoubleValidator(0.1, 100.0, 2))
        self.height_input.setVisible(False)

        self.depth_check = QCheckBox("Depth") # Checkbox for depth measurement
        self.depth_input = QLineEdit()
        self.depth_input.setPlaceholderText("Enter the depth in meters")
        self.depth_input.setValidator(QDoubleValidator(0.1, 100.0, 2))
        self.depth_input.setVisible(False)

        self.volume_check = QCheckBox("Volume")

        self.height_check.toggled.connect(lambda: self.depth_check.setChecked(False) if self.height_check.isChecked() else None)
        self.depth_check.toggled.connect(lambda: self.height_check.setChecked(False) if self.depth_check.isChecked() else None)
        
        # Add all buttons to layout
        self.measurement_buttons_layout.addWidget(self.line_button)
        self.measurement_buttons_layout.addWidget(self.presized_button)
        self.measurement_buttons_layout.addWidget(self.polygon_button)
        self.measurement_buttons_layout.addWidget(self.complete_polygon_button)  # Add complete button
        self.measurement_buttons_layout.addWidget(self.round_pillar_polygon_button)
        self.measurement_buttons_layout.addWidget(self.baseline_button)
        self.measurement_buttons_layout.addWidget(self.inclination_button)
        self.measurement_buttons_layout.addWidget(self.pillar_dimension)
        self.measurement_buttons_layout.addWidget(self.Ohe_angle_button)
        self.measurement_buttons_layout.addWidget(self.all_angle_button)    
        self.measurement_buttons_layout.addWidget(self.defect_angle_button)
        self.measurement_buttons_layout.addWidget(self.elivation_btn_catenary_raillevel_button)
        self.measurement_buttons_layout.addWidget(self.elivation_btn_contact_raillevel_button)
        self.measurement_buttons_layout.addWidget(self.complete_curve_button)  # Add complete button
        self.measurement_buttons_layout.addWidget(self.elivation_btn_catenary_contact_button)
        self.measurement_buttons_layout.addWidget(self.cut_hill_button)
        self.measurement_buttons_layout.addWidget(self.excavate_button)
        
        self.measurement_buttons_layout.addWidget(self.depth_check)
        self.measurement_buttons_layout.addWidget(self.depth_input)

        self.measurement_buttons_layout.addWidget(self.height_check)
        self.measurement_buttons_layout.addWidget(self.height_input)

        self.measurement_buttons_layout.addWidget(self.volume_check)

        # ========== (3) POLYGON-DIGGING CONNECTION ========== #
        # self.create_polygon_digging_connection_section()

        # Add the container to the main layout
        self.measurement_layout.addWidget(self.measurement_buttons_container)
        
        # Initially hide the measurement buttons
        self.measurement_buttons_container.setVisible(False)
        
        self.measurement_group.setLayout(self.measurement_layout)
        self.measurement_group.setEnabled(False)

        # Create a scroll area for the measurement section
        self.measurement_scroll = QScrollArea()
        self.measurement_scroll.setWidgetResizable(True)
        self.measurement_scroll.setWidget(self.measurement_group)
        self.measurement_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        
        # Set a fixed minimum height (e.g., 400 pixels)
        self.measurement_scroll.setMinimumHeight(200)
        self.main_measurement_layout.addWidget(self.measurement_scroll, stretch=3) 
        # self.main_measurement_layout.addWidget(self.measurement_group)

        self.measurement_group.setStyleSheet("""
            QGroupBox {
                border: 1px solid gray;
                border-radius: 5px;
                margin: 0.2em;
                margin-top: 0.5em;
                padding-left: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px;
            }
        """)

# ==================================================================================================================================
# Define function for the Output section:
    def create_action_section(self):
        self.action_group = QGroupBox("Action")
        self.action_layout = QVBoxLayout()
        
        # Remove margins and spacing to minimize empty space
        self.action_layout.setContentsMargins(8, 3, 5, 5) 
        self.action_layout.setSpacing(3)  # Increased spacing between checkboxes
        
        # Create checkboxes directly in the layout (no extra group box)
        self.measurement_check = QCheckBox("Measurement")
        self.measurement_check.setMinimumHeight(30)  # Set minimum height for each checkbox
        self.filling_check = QCheckBox("Filling")
        self.filling_check.setMinimumHeight(30)
        self.cutting_check = QCheckBox("Cutting")
        self.cutting_check.setMinimumHeight(30)
        self.extraction_check = QCheckBox("Extraction")
        self.extraction_check.setMinimumHeight(30)
        self.railway_measurement_check = QCheckBox("Railway Measurement")
        self.railway_measurement_check.setMinimumHeight(30)
        
        # Initially disable all checkboxes (will be enabled after Start Measurement)
        self.measurement_check.setEnabled(False)
        self.filling_check.setEnabled(False)
        self.cutting_check.setEnabled(False)
        self.extraction_check.setEnabled(False)
        self.railway_measurement_check.setEnabled(False)
        
        # Add checkboxes directly to the layout with proper spacing
        self.action_layout.addWidget(self.measurement_check)
        self.action_layout.addSpacing(3)  # Add extra spacing between checkboxes
        self.action_layout.addWidget(self.filling_check)
        self.action_layout.addSpacing(3)  # Add extra spacing between checkboxes
        self.action_layout.addWidget(self.cutting_check)
        self.action_layout.addSpacing(3)  # Add extra spacing between checkboxes
        self.action_layout.addWidget(self.extraction_check)
        self.action_layout.addSpacing(3)  # Add extra spacing between checkboxes
        self.action_layout.addWidget(self.railway_measurement_check)
        
        # ========== DIGGING POINT SECTION (TOP) ========== #
        self.digging_point_input = DiggingPointInput(self)
        self.action_layout.addWidget(self.digging_point_input)
        self.digging_point_input.setVisible(False)
        
        # ========== FINALIZE LAYOUT ========== #
        self.action_group.setLayout(self.action_layout)
        
        # Set fixed height for the action group (200 as requested)
        self.action_group.setFixedHeight(170)
        
        # Add attribute to store cropped data
        self.cropped_cloud = None
        
        # Add the action group directly to the left layout (no scroll area)
        self.main_measurement_layout.addWidget(self.action_group)

        self.action_group.setStyleSheet("""
            QGroupBox {
            border: 1px solid gray;
            border-radius: 5px;
            margin-top: 0.5em;
            }
            QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 3px;
            }
            """)
        
# ==================================================================================================================================   
# Define function for the Output section:
    def create_output_section(self):
        self.output_group = QGroupBox("Output") # Section Name
        self.output_layout = QVBoxLayout()

        # Remove margins and spacing to minimize empty space
        self.output_layout.setContentsMargins(5, 5, 5, 5) 
        self.output_layout.setSpacing(3)
        
        self.output_list = QListWidget()
        self.output_list.setMinimumHeight(150)

        self.output_list.setStyleSheet("""
            QListWidget {
                font-size: 17px;  /* Increase font size for output text */
                font-family: Bold;
            }
            QListWidget::item {
                padding: 5px;  /* Add padding for better readability */
            }
        """)
        
        # Add output list to the group
        self.output_layout.addWidget(self.output_list)
        
        # Create Report Button with dropdown menu
        self.save_layer_button = QPushButton("Save Layer")

        self.output_layout.addWidget(self.save_layer_button)
        
        self.output_group.setLayout(self.output_layout)
        self.main_measurement_layout.addWidget(self.output_group)

        self.output_group.setStyleSheet("""
            QGroupBox {
            border: 1px solid gray;
            border-radius: 5px;
            margin-top: 0.5em;
            }
            QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 3px;
            }
            """)
        
    def setup_scroll_behavior(self):
        """Setup proper scroll behavior for the right section"""
        # Install event filters
        if hasattr(self, 'right_scroll_area') and self.right_scroll_area:
            self.right_scroll_area.viewport().installEventFilter(self)
        # Also install event filters on scale section widgets
        if hasattr(self, 'scale_section') and self.scale_section:
            self.scale_section.installEventFilter(self)
        if hasattr(self, 'volume_slider') and self.volume_slider:
            self.volume_slider.installEventFilter(self)
        if hasattr(self, 'scale_canvas') and self.scale_canvas:
            self.scale_canvas.installEventFilter(self)

    def eventFilter(self, obj, event):
        """Handle mouse wheel events to prevent scrolling when over VTK or matplotlib"""
        
        if event.type() == QEvent.Wheel:
            # Check if this is the scroll area viewport
            if hasattr(self, 'right_scroll_area') and obj == self.right_scroll_area.viewport():
                # Get current mouse position
                mouse_pos = QCursor.pos()
                
                # Check if mouse is over VTK widget
                if self.vtk_widget:
                    vtk_global_rect = self.vtk_widget.frameGeometry()
                    vtk_global_rect.moveTopLeft(self.vtk_widget.mapToGlobal(self.vtk_widget.rect().topLeft()))
                    
                    if vtk_global_rect.contains(mouse_pos):
                        # Mouse is over VTK - block scroll, allow zoom
                        return True
                
                # Check if mouse is over matplotlib canvas
                if self.canvas:
                    canvas_global_rect = self.canvas.frameGeometry()
                    canvas_global_rect.moveTopLeft(self.canvas.mapToGlobal(self.canvas.rect().topLeft()))
                    
                    if canvas_global_rect.contains(mouse_pos):
                        # Mouse is over canvas - block scroll
                        return True
                    
                # NEW: Check if mouse is over scale section
                if hasattr(self, 'scale_section') and self.scale_section:
                    scale_global_rect = self.scale_section.frameGeometry()
                    scale_global_rect.moveTopLeft(self.scale_section.mapToGlobal(self.scale_section.rect().topLeft()))
                    
                    if scale_global_rect.contains(mouse_pos):
                        # Mouse is over scale section - block scroll, allow scale slider to work
                        return True
        
        return super().eventFilter(obj, event)               