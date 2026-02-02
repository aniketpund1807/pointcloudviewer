# Pointcloudviewer.py 
import os
import numpy as np
import open3d as o3d
import time
import json
import matplotlib.path
import math

# PyQt imports
from PyQt5.QtWidgets import (
    QVBoxLayout, QHBoxLayout, QLabel, QWidget, QPushButton, QFileDialog, QMessageBox, QDialog, QCheckBox, QFrame, QGroupBox, QComboBox,
    QInputDialog, QMainWindow, QApplication, QFormLayout, QSizePolicy, QScrollArea
)
from PyQt5.QtCore import Qt, QByteArray, QSize, QRectF, QTimer, QEvent, QPoint
from PyQt5.QtGui import QPixmap, QPainter, QIcon
from PyQt5.QtSvg import QSvgRenderer

# VTK imports
import vtk
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from vtkmodules.vtkFiltersSources import vtkSphereSource, vtkLineSource, vtkPlaneSource
from vtkmodules.vtkInteractionStyle import vtkInteractorStyleTrackballCamera
from vtkmodules.vtkRenderingCore import (vtkActor, vtkPolyDataMapper, vtkRenderer)
from vtkmodules.vtkRenderingFreeType import vtkVectorText
from vtkmodules.vtkFiltersGeneral import vtkTransformPolyDataFilter
# from vtk.util.numpy_support import numpy_to_vtk

from datetime import datetime
from math import sqrt, degrees, acos, atan2
            
from utils import find_best_fitting_plane
from dialogs import (ConstructionConfigDialog, MaterialLineDialog, DesignNewDialog, WorksheetNewDialog, ConstructionNewDialog, HelpDialog,
                     CreateProjectDialog, CurveDialog, ZeroLineDialog, ExistingWorksheetDialog, RoadPlaneWidthDialog, MeasurementNewDialog,
                     MergerLayerConfigDialog, ElevationangleDialog)
from application_ui import ApplicationUI
from measurement_widget import MeasurementWidget
from digging_point import DiggingPointInput

# =======================================================================================================================================
#                                                        ** CLASS POINTCLOUDVIEWER **
# =======================================================================================================================================
class PointCloudViewer(ApplicationUI):
    def __init__(self, username=None):  # Add username parameter
        super().__init__()
        self.current_user = username or "guest"  # Store logged-in user

        # Add these lines
        self.current_worksheet_name = None
        self.current_project_name = None     # <-- Important
        self.current_worksheet_data = {}

        # Curve-related attributes
        self.curve_active = False
        self.curve_start_x = None
        self.curve_annotation = None        # First/main curve label (yellow box)
        self.curve_arrow_annotation = None
        self.curve_pick_id = None
        self.curve_labels = []              # Stores ALL "5.0° - I" labels on top (including intermediates)
        self.current_curve_text = ""        # Stores the text like "5.0° - I"

        self.road_plane_actors = []      # Stores the two side planes (left + right)
        self.road_plane_center_actor = None   # Optional: centre line for reference

        self.slider_marker_actor = None
        self.slider_marker_radius = 0.45

        # NEW: storage for 3D curve actors (so we can clear them later)
        self.curve_3d_actors = []          # list of vtkActor for the yellow arcs
        self.curve_start_point_3d = None   # world coordinate of the point where curve started
        self.curve_start_chainage = None   # distance along zero line at start
        # NEW: 3D curve support
        self.surface_line_3d_points = []


        # NEW: Unified plane support for ALL baselines
        self.baseline_plane_actors = []     # Stores all plane actors (individual + combined)
        self.plane_colors = {               # Distinct semi-transparent colors for each type
            'surface': (0.0, 0.8, 0.0, 0.4),      # Green
            'construction': (1.0, 0.0, 0.0, 0.4), # Red
            'road_surface': (0.0, 0.6, 1.0, 0.45),# Blue
            'deck_line': (1.0, 0.5, 0.0, 0.4),    # Orange
            'projection_line': (0.5, 0.0, 0.5, 0.4), # Purple
            'material': (1.0, 1.0, 0.0, 0.4),     # Yellow
        }

        # List of line types considered as "baselines" for plane mapping
        self.baseline_types = ['surface', 'construction', 'road_surface', 'deck_line', 'projection_line', 'material']

        # Construction mode state
        self.construction_mode_active = False
        self.current_construction_layer = None  # Will store layer name/folder if needed later
        
        # Define worksheet base directory
        self.WORKSHEETS_BASE_DIR = r"D:\3D_Tool\user\worksheets"
        os.makedirs(self.WORKSHEETS_BASE_DIR, exist_ok=True)

        # === ADD THIS: Projects base directory ===
        self.PROJECTS_BASE_DIR = r"D:\3D_Tool\projects"
        os.makedirs(self.PROJECTS_BASE_DIR, exist_ok=True)
        
        # Initialize specific attributes that need different values
        self.start_point = np.array([387211.43846649484, 2061092.3144329898, 598.9991744523196])
        self.end_point = np.array([387219.37847222225, 2060516.8861111111, 612.2502197265625])
        self.total_distance = np.sqrt((self.end_point[0] - self.start_point[0])**2 + (self.end_point[1] - self.start_point[1])**2)
        self.original_total_distance = self.total_distance
        
        # Connect signals
        self.connect_signals()

        self.measurement_widget = MeasurementWidget(self)

        # NEW: Connect label click event AFTER everything is initialized
        self.label_pick_id = None  # Initialize the attribute
        
        # Add this method call to set up the label click handler
        QTimer.singleShot(100, self.setup_label_click_handler)  # Small delay to ensure canvas is ready

        # Create digging point input section
        self.digging_point_input = DiggingPointInput(self)
        self.measurement_layout.addWidget(self.digging_point_input)
        self.digging_point_input.setVisible(False)  # Start hidden

        # =============================================================================================================================================
        # Initialize surface selection group (will be shown when needed)
        self.surface_selection_group = QGroupBox("Surface Selection")
        surface_selection_layout = QHBoxLayout()
        self.surface_combo = QComboBox()
        self.surface_combo.setPlaceholderText("Select Surface")
        surface_selection_layout.addWidget(self.surface_combo)
        self.surface_ok_button = QPushButton("OK")
        self.surface_ok_button.clicked.connect(self.highlight_selected_surface)
        surface_selection_layout.addWidget(self.surface_ok_button)
        self.surface_selection_group.setLayout(surface_selection_layout)
        self.surface_selection_group.setVisible(False)  # Start hidden

# =============================================================================================================================================
        # Digging Point Connection Section (move this from DiggingPointInput to here)
        self.connection_group = QGroupBox("Digging Point Connect")
        self.connection_layout = QFormLayout()
        
        # Create 'Connect Points' button for make connection in digging points:
        self.from_digging_combo = QComboBox()
        self.to_digging_combo = QComboBox()
        self.connect_button = QPushButton("Connect Points")
        self.connect_button.clicked.connect(self.connect_digging_points)

        # Create a dropdownboxes for the selecting digging points from the 'From & To' these two digging dropdown boxes
        self.connection_layout.addRow("From:", self.from_digging_combo)
        self.connection_layout.addRow("To:", self.to_digging_combo)
        self.connection_layout.addRow(self.connect_button)

        self.connection_group.setLayout(self.connection_layout)
        
        # Add these lines to set a minimum height for the connection group
        self.connection_group.setMinimumHeight(150)
        self.connection_group.setFixedWidth(270)  # Adjust this value as needed
        self.connection_group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.MinimumExpanding)

        self.measurement_layout.addWidget(self.connection_group)
        self.connection_group.setVisible(False)
        
        # Connect signals for color changes when selecting digging points
        self.from_digging_combo.currentIndexChanged.connect(self.update_digging_point_colors)
        self.to_digging_combo.currentIndexChanged.connect(self.update_digging_point_colors)

        self.create_polygon_digging_connection_section()

        # Add lock state tracking
        self.is_3d_locked = False
        self.custom_interactor_style = None

        self.merger_plane_actors = {}

        # In your class initialization
        self.reference_actors = []

    def setup_label_click_handler(self):
        """Set up the label click event handler after canvas is fully initialized"""
        if self.canvas:
            self.label_pick_id = self.canvas.mpl_connect('pick_event', self.on_label_click)  

    def on_label_click(self, event):
        """Handle click on construction dot labels"""
        artist = event.artist
        
        # Check if this is a construction dot label
        if hasattr(artist, 'point_data') and artist.point_data.get('type') == 'construction_dot':
            point_number = artist.point_data['point_number']
            x = artist.point_data['x']
            y = artist.point_data['y']
            
            # Calculate chainage for this point using the KM+Interval format
            if self.zero_line_set and hasattr(self, 'zero_start_km') and self.zero_start_km is not None:
                # Calculate which interval this point falls into
                # x is the distance along the zero line in meters
                interval_number = int(x / self.zero_interval) if self.zero_interval > 0 else 0
                
                # Calculate the interval value (multiple of the interval)
                interval_value = interval_number * self.zero_interval
                
                # Format as KM+Interval (3 digits with leading zeros)
                chainage_label = f"{self.zero_start_km}+{interval_value:03d}"
                
                # Add note about exact position if not exactly on interval
                if self.zero_interval > 0:
                    exact_position = x
                    exact_interval_value = int(exact_position / self.zero_interval) * self.zero_interval
                    if abs(exact_position - exact_interval_value) > 0.01:  # If not exactly on interval
                        chainage_label += f" (approx, actual: {exact_position:.2f}m)"
            else:
                chainage_label = f"Distance: {x:.2f}m"
            
            # Open construction configuration dialog with updated chainage format
            dialog = ConstructionConfigDialog(chainage_label, self)
            dialog.setWindowTitle(f"Construction Configuration - Point P{point_number}")
            
            if dialog.exec_() == QDialog.Accepted:
                config = dialog.get_configuration()
                self.message_text.append(f"Construction configuration saved for Point P{point_number}:")
                self.message_text.append(f"  Chainage: {config['chainage']}")
                self.message_text.append(f"  Water Position: {config['water_position']}")
                self.message_text.append(f"  Base Shape: {config['base_shape']}")
                self.message_text.append(f"  Pillar Shape: {config['pillar_shape']}")
                self.message_text.append(f"  Span Shape: {config['span_shape']}")
                self.message_text.append(f"  Base Dimensions: Width={config['base_width']}, Length={config['base_length']}, Height={config['base_height']}")
                self.message_text.append(f"  Pillar Dimensions: Radius={config['pillar_radius']}, Height={config['pillar_height']}")
                self.message_text.append(f"  Span Dimensions: Height={config['span_height']}, Width={config['span_width']}")
                
                # Store this configuration with the point for later reference
                artist.point_data['config'] = config
                
                # Update the label to show it's configured (add checkmark)
                artist.set_text(f'P{point_number}✓')
                artist.set_bbox(dict(
                    boxstyle='round,pad=0.5', 
                    facecolor='lightgreen', 
                    alpha=0.9,
                    edgecolor='green',
                    linewidth=2
                ))
                
                # Redraw the canvas
                self.canvas.draw_idle()

# =======================================================================================================================================
    def scroll_graph_with_slider(self, value):
        """Scroll the graph canvas based on slider position"""
        if not hasattr(self, 'graph_horizontal_scrollbar') or not self.graph_horizontal_scrollbar:
            return
        
        # Get the maximum range of the slider
        slider_max = self.volume_slider.maximum()
        
        # Get the maximum range of the scrollbar
        scrollbar_max = self.graph_horizontal_scrollbar.maximum()
        
        # Calculate the position
        if slider_max > 0 and scrollbar_max > 0:
            # Map slider value (0-100) to scrollbar range
            scroll_position = int((value / slider_max) * scrollbar_max)
            self.graph_horizontal_scrollbar.setValue(scroll_position)
            
            # Update the visual marker on the main graph
            self.update_main_graph_marker(value)
        
# =======================================================================================================================================
# HOVER HANDLER FOR POINTS
    def on_hover(self, event):
        if event.inaxes != self.ax:
            self.annotation.set_visible(False)
            self.canvas.draw_idle()
            return
        vis = self.annotation.get_visible()
        closest_point = None
        min_dist = float('inf')
        for line_type in ['surface', 'construction', 'road_surface']:
            for artist in self.line_types[line_type]['artists']:
                if len(artist.get_xdata()) == 0:
                    continue
                xs = artist.get_xdata()
                ys = artist.get_ydata()
                distances = np.hypot(xs - event.xdata, ys - event.ydata)
                idx = np.argmin(distances)
                if distances[idx] < 0.2: # threshold for hover detection
                    if distances[idx] < min_dist:
                        min_dist = distances[idx]
                        closest_point = (xs[idx], ys[idx])
        if closest_point is not None:
            x_dist, rel_elev = closest_point
            if self.zero_line_set and self.total_distance > 0:
                t = x_dist / self.total_distance
                dir_vec = self.zero_end_point - self.zero_start_point
                pos_along = self.zero_start_point + t * dir_vec
                abs_z = self.zero_start_z + rel_elev
                text = f'({pos_along[0]:.3f}, {pos_along[1]:.3f}, {abs_z:.2f})'
            else:
                text = f'({x_dist:.2f}, {rel_elev:.2f})'
            self.annotation.xy = closest_point
            self.annotation.set_text(text)
            self.annotation.set_visible(True)
            self.canvas.draw_idle()
        elif vis:
            self.annotation.set_visible(False)
            self.canvas.draw_idle()
    
# =======================================================================================================================================
    def reset_measurement_tools(self):
        """Clear all measurement buttons and states"""
        self.line_button.setVisible(False)
        self.polygon_button.setVisible(False)
        self.stockpile_polygon_button.setVisible(False)
        self.complete_polygon_button.setVisible(False)
        self.presized_button.setVisible(False)
        self.metrics_group.setVisible(False)

# =======================================================================================================================================
# Close dropdown when clicking outside
    def eventFilter(self, obj, event):
        if event.type() == event.MouseButtonPress:
            if obj.isWidgetType() and obj.windowFlags() & Qt.Popup:
                # click outside any popup → close all popups
                for btn in [self.worksheet_button, self.design_button,
                            self.construction_button, self.measurement_button]:
                    if btn.isChecked():
                        btn.setChecked(False)
                        btn.property("dropdown").hide()
        return super().eventFilter(obj, event)
    
# =======================================================================================================================================
# Helper called when user picks New / Existing (optional)
    def on_dropdown_choice(self, main_btn, choice):
        main_btn.setChecked(False)
        main_btn.property("dropdown").hide()
        #print(f"{main_btn.text()} → {choice} selected")   # replace with real logic

# =======================================================================================================================================   
    def load_last_worksheet(self):
        """Load and display the most recently created worksheet on startup"""
        if not os.path.exists(self.WORKSHEET_FILE):
            return
        try:
            with open(self.WORKSHEET_FILE, 'r', encoding='utf-8') as f:
                lines = [line.strip() for line in f if line.strip()]
                if not lines:
                    return
                last_data = json.loads(lines[-1])
                self.display_current_worksheet(last_data)
                self.message_text.append(f"Loaded last worksheet: {last_data.get('worksheet_name', 'Unknown')}")
        except Exception as e:
            print(f"Failed to load last worksheet: {e}")

# =======================================================================================================================================
# TOGGLE MESSAGE SECTION
    def toggle_message_section(self):
        self.message_visible = not self.message_visible
        self.message_section.setVisible(self.message_visible)
        self.message_button.setText("Hide Message" if self.message_visible else "Message")

# =======================================================================================================================================
    def toggle_worksheet_options(self):
        checked = self.worksheet_button.isChecked()
        self.sub_buttons_widget.setVisible(checked)
        
        # Optional: change icon/text to indicate open/close state
        self.worksheet_button.setText("📊Worksheet ▼" if checked else "📊Worksheet")

# =======================================================================================================================================
    def toggle_design_options(self):
        checked = self.design_button.isChecked()
        self.sub_design_buttons_widget.setVisible(checked)
        
        # Optional: change icon/text to indicate open/close state
        self.design_button.setText("📐Design ▼" if checked else "📐Design")

# =======================================================================================================================================   
    def toggle_construction_options(self):
        checked = self.construction_button.isChecked()
        self.sub_construction_buttons_widget.setVisible(checked)
        
        # Optional: change icon/text to indicate open/close state
        self.construction_button.setText("🏗 Construction ▼" if checked else "🏗 Construction")

# =======================================================================================================================================
    def toggle_measurement_options(self):
        checked = self.measurement_button.isChecked()
        self.sub_measurement_buttons_widget.setVisible(checked)
        
        # Optional: change icon/text to indicate open/close state
        self.measurement_button.setText("📏Measurement ▼" if checked else "📏Measurement")

# =======================================================================================================================================
    def open_create_project_dialog(self):
        dialog = CreateProjectDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            data = dialog.get_data()
            project_name = data["project_name"].strip()
            pointcloud_files = data["pointcloud_files"]
            category = data["category"]

            if not project_name:
                QMessageBox.warning(self, "Error", "Project name is required!")
                return

            if not pointcloud_files:
                QMessageBox.warning(self, "Error", "Please select at least one point cloud file or folder.")
                return

            # Define base projects directory
            BASE_PROJECTS_DIR = r"D:\3D_Tool\projects"
            os.makedirs(BASE_PROJECTS_DIR, exist_ok=True)

            # Create project-specific folder
            project_folder = os.path.join(BASE_PROJECTS_DIR, project_name)
            try:
                os.makedirs(project_folder, exist_ok=False)  # Raises error if already exists
            except FileExistsError:
                reply = QMessageBox.question(
                    self, "Project Exists",
                    f"A project named '{project_name}' already exists.\nDo you want to overwrite it?",
                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No
                )
                if reply == QMessageBox.No:
                    return
                # If yes, continue (folder already exists, we'll overwrite config)

            # Prepare project config data
            project_entry = {
                "project_name": project_name,
                "pointcloud_files": pointcloud_files,  # List of full paths
                "category": category,
                "created_at": datetime.now().isoformat(),
                "created_by": self.current_user  # Optional: track who created it
            }

            # Path to project_config.txt inside the project folder
            config_file_path = os.path.join(project_folder, "project_config.txt")

            try:
                with open(config_file_path, 'w', encoding='utf-8') as f:
                    json.dump(project_entry, f, indent=4)
                
                QMessageBox.information(
                    self, "Success",
                    f"Project '{project_name}' created successfully!\n\n"
                    f"Location:\n{project_folder}\n\n"
                    f"Point cloud files linked: {len(pointcloud_files)} file(s)"
                )

                self.message_text.append(f"New project created: {project_name}")
                self.message_text.append(f"   → Folder: {project_folder}")
                self.message_text.append(f"   → Files linked: {len(pointcloud_files)}")

                # Optional: You can now allow loading this project later
                # Or auto-load point clouds if desired

            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save project configuration:\n{str(e)}")
                self.message_text.append(f"Error saving project '{project_name}': {str(e)}")

# =======================================================================================================================================
    def open_new_worksheet_dialog(self):
        """Open dialog to create a new worksheet with Road/Bridge baseline selection and dynamic UI updates.
        Creates 'designs' or 'measurements' subfolder based on 2D/3D selection, then adds the initial layer folder."""
        dialog = WorksheetNewDialog(self)
        if dialog.exec_() != QDialog.Accepted:
            return

        data = dialog.get_data()
        worksheet_name = data["worksheet_name"].strip()

        if not worksheet_name:
            QMessageBox.warning(self, "Invalid Name", "Worksheet name cannot be empty!")
            return

        project_name = data["project_name"] if data["project_name"].strip() else None
        category = data["worksheet_category"]  # "Road", "Bridge", "Other", "None"
        worksheet_type = data["worksheet_type"]  # "Design" or "Measurement"
        dimension = "2D" if worksheet_type == "Design" else "3D"
        initial_layer_name = data["initial_layer_name"].strip()  # User-entered layer name
        point_cloud_file = data.get("point_cloud_file")

        # Create main worksheet folder
        worksheet_folder = os.path.join(self.WORKSHEETS_BASE_DIR, worksheet_name)
        try:
            os.makedirs(worksheet_folder, exist_ok=False)
        except FileExistsError:
            reply = QMessageBox.question(self, "Folder Exists",
                                        f"A worksheet named '{worksheet_name}' already exists.\n"
                                        f"Do you want to overwrite it?",
                                        QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply == QMessageBox.No:
                return
            # Continue to overwrite (we'll recreate subfolders)

        # === NEW LOGIC: Create 'designs' or 'measurements' based on dimension ===
        if dimension == "2D":
            base_subfolder = os.path.join(worksheet_folder, "designs")
            self.message_text.append("   → Creating 'designs' folder for 2D Design mode")
        else:  # 3D
            base_subfolder = os.path.join(worksheet_folder, "measurements")
            self.message_text.append("   → Creating 'measurements' folder for 3D Measurement mode")

        try:
            os.makedirs(base_subfolder, exist_ok=True)
        except Exception as e:
            self.message_text.append(f"   → Warning: Could not create subfolder '{os.path.basename(base_subfolder)}': {str(e)}")

        # === Create initial layer folder inside the correct subfolder (only if layer name provided) ===
        layer_folder = None
        if initial_layer_name:
            layer_folder = os.path.join(base_subfolder, initial_layer_name)
            try:
                os.makedirs(layer_folder, exist_ok=True)
                self.message_text.append(f"   → Initial layer folder created: {initial_layer_name}")
                self.message_text.append(f"      Path: {layer_folder}")
            except Exception as e:
                self.message_text.append(f"   → Error creating layer folder '{initial_layer_name}': {str(e)}")
                layer_folder = None
        else:
            self.message_text.append("   → No layer name entered — no initial layer folder created")

        # Save worksheet config
        config_data = {
            "worksheet_name": worksheet_name,
            "project_name": project_name or "None",
            "created_at": datetime.now().isoformat(),
            "created_by": self.current_user,
            "worksheet_type": worksheet_type,
            "initial_layer": initial_layer_name or None,
            "worksheet_category": category,
            "dimension": dimension,
            "point_cloud_file": point_cloud_file
        }

        config_path = os.path.join(worksheet_folder, "worksheet_config.txt")
        try:
            with open(config_path, 'w', encoding='utf-8') as f:
                json.dump(config_data, f, indent=4)
        except Exception as e:
            QMessageBox.critical(self, "Save Failed", f"Could not save worksheet config:\n{str(e)}")
            return
        
        # Update application state
        self.current_worksheet_name = worksheet_name
        self.current_project_name = project_name
        self.current_worksheet_data = config_data.copy()
        self.display_current_worksheet(config_data)

        # Add initial layer to left panel (if name was provided)
        if initial_layer_name:
            self.add_layer_to_panel(initial_layer_name, dimension)

        # Show/Hide 3D vs 2D sections
        if dimension == "3D":
            self.three_D_frame.setVisible(True)
            self.two_D_frame.setVisible(False)
            self.main_measurement_section.setVisible(True)
            self.right_section.setMinimumHeight(900)
            self.message_text.append("→ 3D Layers section shown, 2D Layers hidden (Measurement mode)")
        elif dimension == "2D":
            self.three_D_frame.setVisible(False)
            self.two_D_frame.setVisible(True)
            self.right_section.setMinimumHeight(1200)
            self.message_text.append("→ 2D Layers section shown, 3D Layers hidden (Design mode)")

        # Log messages
        self.message_text.append(f"Worksheet '{worksheet_name}' created successfully!")
        self.message_text.append(f"   → Path: {worksheet_folder}")
        self.message_text.append(f"   → Type: {worksheet_type} ({dimension})")
        self.message_text.append(f"   → Category: {category}")

        if point_cloud_file:
            self.message_text.append(f"   → Point Cloud Linked: {os.path.basename(point_cloud_file)}")
        else:
            self.message_text.append(f"   → Point Cloud: None selected")

        # Baseline logic (Road/Bridge)
        ref_type = None
        ref_line = None
        if category in ["Road", "Bridge"]:
            ref_type = category
            available_lines = ["Road_Layer_1", "Road_Layer_2", "Road_Layer_3"] if category == "Road" \
                              else ["Bridge_Layer_1", "Bridge_Layer_2", "Bridge_Layer_3"]
            ref_line = available_lines[0] if available_lines else None

            self.message_text.append(f"   → Baseline Type: {ref_type}")
            if ref_line:
                self.message_text.append(f"   → Reference Baseline: {ref_line}")

        # UI updates based on category
        self.bottom_section.setVisible(category in ["Road", "Bridge"])

        # Hide all containers first
        self.surface_container.setVisible(False)
        self.construction_container.setVisible(False)
        self.road_surface_container.setVisible(False)
        self.zero_container.setVisible(False)
        self.deck_line_container.setVisible(False)
        self.projection_container.setVisible(False)
        self.construction_dots_container.setVisible(False)
        self.bridge_zero_container.setVisible(False)

        if category == "Road":
            self.surface_container.setVisible(True)
            self.construction_container.setVisible(True)
            self.road_surface_container.setVisible(True)
            self.zero_container.setVisible(True)
            self.message_text.append(f"Mode Activated: {dimension} - ROAD Mode")
        elif category == "Bridge":
            self.deck_line_container.setVisible(True)
            self.projection_container.setVisible(True)
            self.construction_dots_container.setVisible(True)
            if hasattr(self, 'bridge_zero_container'):
                self.bridge_zero_container.setVisible(True)
            self.message_text.append(f"Mode Activated: {dimension} - BRIDGE Mode")
        else:
            self.message_text.append(f"Mode Activated: {dimension} - General Mode")

        if ref_line:
            self.message_text.append(f"Reference Baseline Set: {ref_line}")

        # Action buttons
        self.preview_button.setVisible(True)
        self.elivation_angle_button.setVisible(True)
        self.threed_map_button.setVisible(True)
        self.save_button.setVisible(True)

        # Auto-check zero lines
        if not self.zero_line_set:
            self.zero_line.setChecked(True)
            if hasattr(self, 'bridge_zero_line'):
                self.bridge_zero_line.setChecked(True)

        # Refresh rendering
        self.canvas.draw()
        if hasattr(self, 'vtk_widget'):
            self.vtk_widget.GetRenderWindow().Render()

        # Auto-load point cloud if selected
        if point_cloud_file and os.path.exists(point_cloud_file):
            self.message_text.append("Auto-loading selected point cloud...")
            success = self.load_point_cloud_from_path(point_cloud_file)
            if success:
                self.message_text.append("Point cloud loaded successfully!")
            else:
                self.message_text.append("Failed to auto-load point cloud.")
        elif point_cloud_file:
            self.message_text.append(f"Selected point cloud file not found: {point_cloud_file}")

        # Final success message
        layer_display = initial_layer_name if initial_layer_name else "(none)"
        layer_path_display = layer_folder if layer_folder else "(not created)"
        pc_display = os.path.basename(point_cloud_file) if point_cloud_file else "None"

        QMessageBox.information(self, "Success",
                                f"Worksheet '{worksheet_name}' created successfully!\n\n"
                                f"Dimension: {dimension} ({worksheet_type})\n"
                                f"Initial Layer: {layer_display}\n"
                                f"Layer Path: {layer_path_display}\n"
                                f"Mode: {category or 'General'}\n"
                                f"Point Cloud: {pc_display}\n\n"
                                f"Saved in:\n{worksheet_folder}")

# =======================================================================================================================================
    def display_current_worksheet(self, config_data):
        """
        Updates the Current Worksheet box and the top mode banner (Measurement/Design)
        """
        if not config_data:
            self.worksheet_display.setVisible(False)
            self.mode_banner.setVisible(False)
            return

        self.worksheet_display.setVisible(True)
        self.mode_banner.setVisible(True)

        worksheet_name = config_data.get("worksheet_name", "Unknown")
        project_name = config_data.get("project_name", "None")
        worksheet_type = config_data.get("worksheet_type", "Unknown")  # "Measurement" or "Design"
        category = config_data.get("worksheet_category", "None")
        created_at = config_data.get("created_at", "Unknown")

        # Format created time
        try:
            from datetime import datetime
            dt = datetime.fromisoformat(created_at.replace("Z", "+00:00"))
            created_str = dt.strftime("%Y-%m-%d %H:%M:%S")
        except:
            created_str = created_at

        # Update worksheet info inside the box
        info_lines = [
            f"<b>Worksheet Name:</b> {worksheet_name}",
            f"<b>Worksheet Type:</b> {worksheet_type}",
            f"<b>Project:</b> {project_name}",
            f"<b>Worksheet Category:</b> {category}",
            f"<b>Worksheet Created:</b> {created_str}"
        ]
        info_text = "<br>".join(info_lines)
        self.worksheet_info_label.setText(info_text)

        # Update title with Active:
        self.worksheet_display.setTitle(f"Active: {worksheet_name}")

        # === UPDATE TOP MODE BANNER ===
        if worksheet_type == "Measurement":
            self.mode_banner.setText("MEASUREMENT MODE")
            self.mode_banner.setStyleSheet("""
            QLabel {
                    font-weight: bold;}
            """)
            # self.mode_banner.setStyleSheet("""
            #     QLabel {
            #         font-size: 22px;
            #         font-weight: bold;
            #         color: white;
            #         background-color: #7B1FA2;
            #         border-radius: 14px;
            #         padding: 16px;
            #         margin: 12px 20px 20px 20px;
            #         border: 4px solid #4A148C;
            #     }
            # """)
        elif worksheet_type == "Design":
            self.mode_banner.setText("DESIGN MODE")
            self.mode_banner.setStyleSheet("""
            QLabel {
                    font-weight: bold;}
            """)
            # self.mode_banner.setStyleSheet("""
            #     QLabel {
            #         font-size: 22px;
            #         font-weight: bold;
            #         color: white;
            #         background-color: #2E7D32;
            #         border-radius: 14px;
            #         padding: 16px;
            #         margin: 12px 20px 20px 20px;
            #         border: 4px solid #1B5E20;
            #     }
            # """)
        else:
            self.mode_banner.setText("GENERAL MODE")
            # self.mode_banner.setStyleSheet("""
            #     QLabel {
            #         font-size: 20px;
            #         font-weight: bold;
            #         color: white;
            #         background-color: #666666;
            #         border-radius: 12px;
            #         padding: 14px;
            #         margin: 10px 15px 15px 15px;
            #     }
            # """)

# =======================================================================================================================================
# OPEN CREATE NEW DESIGN LAYER DIALOG
    def open_create_new_design_layer_dialog(self):
        """Open the Design New Layer dialog and save config to current worksheet's designs folder"""
        if not hasattr(self, 'current_worksheet_name') or not self.current_worksheet_name:
            QMessageBox.warning(self, "No Active Worksheet",
                                "Please create or open a worksheet first before creating a design layer.")
            return

        dialog = DesignNewDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            config = dialog.get_configuration()
            layer_name = config["layer_name"]
            dimension = config["dimension"]
            ref_type = config["reference_type"]
            ref_line = config["reference_line"]

            # Build path inside current worksheet
            base_designs_path = os.path.join(self.WORKSHEETS_BASE_DIR, self.current_worksheet_name, "designs")
            layer_folder = os.path.join(base_designs_path, layer_name)

            # Create folder (safe because we already checked in dialog)
            os.makedirs(layer_folder, exist_ok=True)

            # Prepare full config data
            full_config = {
                "layer_name": layer_name,
                "dimension": dimension,
                "reference_type": ref_type,
                "reference_line": ref_line,
                "project_name": getattr(self, 'current_project_name', 'None'),
                "worksheet_name": self.current_worksheet_name,
                "created_by": self.current_user,
                "created_at": datetime.now().isoformat(),
            }

            # Save config file
            config_file_path = os.path.join(layer_folder, "design_layer_config.txt")
            try:
                with open(config_file_path, 'w', encoding='utf-8') as f:
                    json.dump(full_config, f, indent=4)

                QMessageBox.information(self, "Success",
                                        f"Design layer '{layer_name}' created successfully!\n\n"
                                        f"Location:\n{layer_folder}")

                self.message_text.append(f"New design layer created: {layer_name}")
                self.message_text.append(f" → Path: {layer_folder}")
                self.message_text.append(f" → Dimension: {dimension}")
                if ref_type:
                    self.message_text.append(f" → Type: {ref_type} ({ref_line or 'No reference'})")

            except Exception as e:
                QMessageBox.critical(self, "Save Failed", f"Could not save design layer config:\n{str(e)}")
                return
            
            # Add to left panel
            if dimension == "2D":
                self.two_D_frame.setVisible(True)
            else: 
                self.three_D_frame.setVisible(True)
            self.add_layer_to_panel(layer_name, dimension)

            # === UI Updates ===
            self.right_section.setMinimumHeight(1200)
            self.bottom_section.setVisible(True)
            self.surface_container.setVisible(False)
            self.construction_container.setVisible(False)
            self.road_surface_container.setVisible(False)
            self.zero_container.setVisible(False)
            self.deck_line_container.setVisible(False)
            self.projection_container.setVisible(False)
            self.construction_dots_container.setVisible(False)
            self.bridge_zero_container.setVisible(False)

            # ========== CRITICAL FIX: Hide construction-specific elements ==========
            self.add_material_line_button.setVisible(False)  # Hide Add Material Line button
            self.material_line_container.setVisible(False)   # Hide Material Line container

            if ref_type == "Road":
                self.surface_container.setVisible(True)
                self.construction_container.setVisible(True)
                self.road_surface_container.setVisible(True)
                self.zero_container.setVisible(True)
                self.message_text.append(f"Design Layer Created: {dimension} - ROAD Mode")
                # Set mode banner for road design
                if hasattr(self, 'mode_banner'):
                    self.mode_banner.setText("DESIGN MODE - ROAD")
                    self.mode_banner.setStyleSheet("""
                        QLabel {

                            font-weight: bold;
                        }
                    """)
                    self.mode_banner.setVisible(True)
            elif ref_type == "Bridge":
                self.deck_line_container.setVisible(True)
                self.projection_container.setVisible(True)
                self.construction_dots_container.setVisible(True)
                self.bridge_zero_container.setVisible(True)
                self.message_text.append(f"Design Layer Created: {dimension} - BRIDGE Mode")
                # Set mode banner for bridge design
                if hasattr(self, 'mode_banner'):
                    self.mode_banner.setText("DESIGN MODE - BRIDGE")
                    self.mode_banner.setStyleSheet("""
                        QLabel {

                            font-weight: bold;
                        }
                    """)
                    self.mode_banner.setVisible(True)
            else:
                self.message_text.append(f"Design Layer Created: {dimension} - No reference type selected")

            if ref_line:
                self.message_text.append(f"Reference Line: {ref_line}")

            # Show action buttons
            self.preview_button.setVisible(True)
            self.elivation_angle_button.setVisible(True)
            self.threed_map_button.setVisible(True)
            self.save_button.setVisible(True)

            # Auto-check zero line
            if not self.zero_line_set:
                self.zero_line.setChecked(True)
                if hasattr(self, 'bridge_zero_line'):
                    self.bridge_zero_line.setChecked(True)

            self.canvas.draw()
            self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================
    def open_construction_new_dialog(self):
        """Handle 'Construction → New' – full workflow: validation, folder creation, config save, UI switch"""
        # 1. Must have an active worksheet
        if not hasattr(self, 'current_worksheet_name') or not self.current_worksheet_name:
            QMessageBox.warning(
                self,
                "No Active Worksheet",
                "Please create or open a worksheet first before creating a construction layer."
            )
            self.message_text.append("Attempted to create construction layer without active worksheet.")
            return

        # 2. Open dialog
        dialog = ConstructionNewDialog(self)
        # Critical: Pass the current worksheet path
        dialog.set_current_worksheet_path(
            os.path.join(self.WORKSHEETS_BASE_DIR, self.current_worksheet_name)
        )
        if dialog.exec_() != QDialog.Accepted:
            return

        data = dialog.get_data()
        layer_name = data['layer_name'].strip()

        if not layer_name:
            QMessageBox.warning(self, "Invalid Name", "Construction layer name cannot be empty!")
            return

        # Determine type
        is_road = data['is_road']
        is_bridge = data['is_bridge']
        construction_type = "Road" if is_road else "Bridge" if is_bridge else "General"

        # 3. Create folder structure
        construction_base = os.path.join(
            self.WORKSHEETS_BASE_DIR,
            self.current_worksheet_name,
            "construction"
        )
        layer_folder = os.path.join(construction_base, layer_name)

        if os.path.exists(layer_folder):
            QMessageBox.warning(
                self,
                "Name Exists",
                f"A construction layer named '{layer_name}' already exists.\n"
                "Please choose a different name."
            )
            return

        try:
            os.makedirs(layer_folder, exist_ok=False)

            # 4. Save config file
            config_data = {
                "construction_layer_name": layer_name,
                "worksheet_name": self.current_worksheet_name,
                "project_name": self.current_project_name or "None",
                "worksheet_type": self.current_worksheet_data.get("worksheet_type", "Unknown"),
                "worksheet_category": self.current_worksheet_data.get("worksheet_category", "None"),
                "construction_type": construction_type,
                "reference_layer_2d": data['reference_layer'],
                "base_lines_reference": data['base_lines_layer'],
                "created_at": datetime.now().isoformat(),
                "created_by": self.current_user,
                "material_lines": []  # will be populated later
            }

            config_path = os.path.join(layer_folder, "Construction_Layer_config.txt")
            with open(config_path, 'w', encoding='utf-8') as f:
                json.dump(config_data, f, indent=4, ensure_ascii=False)

            # 5. Store in memory for current session
            self.current_mode = "road" if is_road else "bridge"
            self.construction_layer_info = {
                "name": layer_name,
                "type": "road" if is_road else "bridge",
                "reference_layer": data['reference_layer'],
                "base_line": data['base_lines_layer'],
                "material_lines": [],
                "folder": layer_folder
            }
            self.construction_layer_created = True
            self.current_construction_layer = layer_name  # optional: track active one

            # 6. Update UI 
            self.right_section.setMinimumHeight(1200)
            self.bottom_section.setVisible(True)

            # 7. Hide all standard line containers
            containers_to_hide = [
                self.zero_container,
                self.surface_container,
                self.construction_container,
                self.road_surface_container,
                self.bridge_zero_container,
                self.projection_container,
                self.construction_dots_container,
                self.deck_line_container,
                # Note: material_line_container will be shown via "Add Material Line" button
            ]
            for container in containers_to_hide:
                if hasattr(self, container.__class__.__name__) or container:
                    try:
                        container.setVisible(False)
                    except:
                        pass

            # Hide standard buttons
            self.preview_button.setVisible(False)
            self.elivation_angle_button.setVisible(False)
            self.threed_map_button.setVisible(False)
            # save_button will be shown below

            # 8. Show construction-specific controls
            self.add_material_line_button.setVisible(True)
            self.save_button.setVisible(True)

            # Optional: make Save button say "Save Construction"
            self.save_button.setText("Save Construction")

            # 9. Add to layers panel
            self.add_layer_to_panel(layer_name, "2D")
            self.two_D_frame.setVisible(True)

            if hasattr(self, 'mode_banner'):
                self.mode_banner.setText("CONSTRUCTION MODE")
                self.mode_banner.setStyleSheet("""
                    QLabel {

                        font-weight: bold;
                    }
                """)
                self.mode_banner.setVisible(True)

            # 10. Feedback
            self.message_text.append(f"Construction layer '{layer_name}' ({construction_type}) created successfully!")
            self.message_text.append(f"   → Folder: {layer_folder}")
            self.message_text.append(f"   → Reference 2D Layer: {data['reference_layer']}")
            self.message_text.append(f"   → Base Line Reference: {data['base_lines_layer']}")
            self.message_text.append("You can now add material lines using the button below.")

            QMessageBox.information(
                self,
                "Construction Layer Created",
                f"<b>{layer_name}</b> ({construction_type})<br><br>"
                f"Reference Layer: {data['reference_layer']}<br>"
                f"Base Line: {data['base_lines_layer']}<br><br>"
                f"Folder created at:<br>{layer_folder}<br><br>"
                f"You can now add material lines and save the construction design."
            )

        except Exception as e:
            QMessageBox.critical(
                self,
                "Creation Failed",
                f"Failed to create construction layer '{layer_name}':\n\n{str(e)}"
            )
            self.message_text.append(f"Error creating construction layer: {str(e)}")

# =======================================================================================================================================
    def switch_to_construction_mode(self):
        """Update bottom section for Construction mode: hide baselines, show only Save button"""
        # Hide ALL baseline-related containers
        self.surface_container.setVisible(False)
        self.road_surface_container.setVisible(False)
        self.zero_container.setVisible(False)
        self.construction_container.setVisible(False)
        self.deck_line_container.setVisible(False)
        self.projection_container.setVisible(False)
        
        if hasattr(self, 'bridge_zero_container'):
            self.bridge_zero_container.setVisible(False)
        if hasattr(self, 'construction_dots_container'):
            self.construction_dots_container.setVisible(False)

        # Hide preview and 3D mapping buttons (not used in construction)
        self.preview_button.setVisible(False)
        self.elivation_angle_button.setVisible(False)
        self.threed_map_button.setVisible(False)

        # Show ONLY the Save button
        self.save_button.setVisible(True)

        # Hide scale section (chainage not relevant in pure construction)
        self.scale_section.setVisible(False)

        # Update message and mode banner
        self.message_text.append("Switched to Construction mode. Only Save button is available.")
        
        if hasattr(self, 'mode_banner'):
            self.mode_banner.setText("CONSTRUCTION MODE")
            self.mode_banner.setStyleSheet("""
                QLabel {

                    font-weight: bold;
                }
            """)
            self.mode_banner.setVisible(True)

        self.canvas.draw_idle()

# =======================================================================================================================================
# OPEN MATERIAL LINE DIALOG WHEN "Construction → New" IS CLICKED
    def open_material_line_dialog(self):
        """Opens the Material Line Configuration Dialog to add a new line"""
        dialog = MaterialLineDialog(parent=self)
        if dialog.exec_() == QDialog.Accepted:
            config = dialog.get_material_data()
            
            # Add visibility flag
            config['visible'] = True
            
            # Store config
            self.material_configs.append(config)
            
            # Create UI entry for the material line
            self.create_material_line_entry(config)
            
            # If this is the first material line, change button color to orange
            if len(self.material_configs) == 1:
                self.add_material_line_button.setStyleSheet("""
                    QPushButton {
                        background-color: #FFA500;
                        color: white;
                        border: none;
                        padding: 10px;
                        border-radius: 5px;
                        font-weight: bold;
                    }
                    QPushButton:hover {
                        background-color: #FF8C00;
                    }
                    QPushButton:pressed {
                        background-color: #FF7F00;
                    }
                """)
            
            self.message_text.append("Material Line Created Successfully!")
            self.message_text.append(f"Name: {config['name']}")
            self.message_text.append(f"Initial Thickness: {config['initial_filling']} mm")
            self.message_text.append(f"Final Thickness: {config['final_compressed']} mm")
        else:
            self.message_text.append("Material Line creation cancelled.")

# =======================================================================================================================================    
# Optional: allow editing later with pencil button
    def edit_material_line(self):
        if not hasattr(self, 'material_configs') or not self.material_configs:
            QMessageBox.information(self, "No Data", "No material line has been created yet.")
            return
        dialog = MaterialLineDialog(material_data=self.material_configs[-1], parent=self)
        if dialog.exec_() == QDialog.Accepted:
            self.material_configs[-1] = dialog.get_material_data()
            self.message_text.append("Material Line configuration updated.")

# =======================================================================================================================================
    def create_material_line_entry(self, material_data, edit_index=None):
        """Create or update a material line entry in the UI (below the Add button)"""
        # Create container
        container = QWidget()
        container.setFixedWidth(250)
        container.setStyleSheet("""
            QWidget {
                background-color: #FFA500;  /* Orange background */
                border: 1px solid #FF8C00;
                border-radius: 5px;
                margin: 3px;
                padding: 5px;
            }
        """)
        
        layout = QHBoxLayout(container)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)
        
        # Checkbox for visibility/selection
        checkbox = QCheckBox()
        checkbox.setChecked(material_data.get('visible', True))
        checkbox.setStyleSheet("""
            QCheckBox {
                background-color: transparent;
                border: none;
            }
            QCheckBox::indicator {
                width: 16px;
                height: 16px;
            }
        """)
        
        # Connect checkbox to toggle visibility
        checkbox.stateChanged.connect(lambda state, idx=len(self.material_items) if edit_index is None else edit_index: 
                                    self.toggle_material_line_visibility(idx, state))
        
        # Material label
        material_label = QLabel(material_data['name'])
        material_label.setStyleSheet("""
            QLabel {
                background-color: transparent;
                border: none;
                padding: 2px;
                font-weight: bold;
                font-size: 14px;
                color: #000000;
            }
        """)
        
        # Pencil button for editing
        pencil_button = QPushButton()
        svg_data = QByteArray()
        svg_data.append(self.PENCIL_SVG)
        renderer = QSvgRenderer(svg_data)
        pixmap = QPixmap(20, 20)
        pixmap.fill(Qt.transparent)
        painter = QPainter(pixmap)
        renderer.render(painter, QRectF(pixmap.rect()))
        painter.end()
        icon = QIcon(pixmap)
        pencil_button.setIcon(icon)
        pencil_button.setIconSize(QSize(20, 20))
        pencil_button.setFixedSize(28, 28)
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
        pencil_button.clicked.connect(lambda checked, idx=len(self.material_items) if edit_index is None else edit_index: 
                                    self.edit_existing_material_line(idx))
        
        # Add widgets to layout
        layout.addWidget(checkbox)
        layout.addWidget(material_label, 1)
        layout.addWidget(pencil_button)
        
        if edit_index is not None:
            # Replace existing item
            if edit_index < len(self.material_items):
                old_container = self.material_items[edit_index]['container']
                old_container.setParent(None)
                old_container.deleteLater()
                
                self.material_items[edit_index] = {
                    'container': container,
                    'checkbox': checkbox,
                    'label': material_label,
                    'pencil_button': pencil_button,
                    'data': material_data
                }
                
                # Re-insert at correct position
                self.material_items_layout.insertWidget(edit_index, container)
        else:
            # Add new item
            new_item = {
                'container': container,
                'checkbox': checkbox,
                'label': material_label,
                'pencil_button': pencil_button,
                'data': material_data
            }
            self.material_items.append(new_item)
            self.material_items_layout.addWidget(container)
        
        # Message for new items only
        if edit_index is None:
            self.message_text.append(f"Added material line: {material_data['name']}")
            
# =======================================================================================================================================
    def edit_existing_material_line(self, index):
        """Edit an existing material line by index"""
        if index < 0 or index >= len(self.material_items):
            return
        
        item = self.material_items[index]
        current_data = item['data']
        
        dialog = MaterialLineDialog(material_data=current_data, parent=self)
        
        if dialog.exec_() == QDialog.Accepted:
            new_material_data = dialog.get_material_data()
            
            # Preserve visibility
            new_material_data['visible'] = current_data.get('visible', True)
            
            # Update stored config as well
            if index < len(self.material_configs):
                self.material_configs[index] = new_material_data
            
            # Update UI
            self.create_material_line_entry(new_material_data, edit_index=index)
            
            self.message_text.append(f"Updated material line: {new_material_data['name']}")

# =======================================================================================================================================
    def toggle_material_line_visibility(self, index, state):
        """Toggle visibility of a material line"""
        if 0 <= index < len(self.material_items):
            item = self.material_items[index]
            is_visible = state == Qt.Checked
            item['data']['visible'] = is_visible
            if index < len(self.material_configs):
                self.material_configs[index]['visible'] = is_visible
            
            if is_visible:
                self.message_text.append(f"Showing material line: {item['data']['name']}")
            else:
                self.message_text.append(f"Hiding material line: {item['data']['name']}")

# =======================================================================================================================================
    def save_material_data(self):
        """Save all material lines"""
        if hasattr(self, 'material_items') and self.material_items:
            self.message_text.append("<b>Material lines saved successfully!</b>")
            self.message_text.append(f"Total: {len(self.material_items)} material line(s)\n")
            for i, item in enumerate(self.material_items):
                data = item['data']
                visibility = "Visible" if data.get('visible', True) else "Hidden"
                self.message_text.append(f"<b>{i+1}. {data['name']}</b> [{visibility}]")
                self.message_text.append(f"   • Initial Thickness: {data['initial_filling']} mm")
                self.message_text.append(f"   • Final Thickness: {data['final_compressed']} mm")
                ref = data.get('ref_layer') or data.get('ref_line') or 'None'
                self.message_text.append(f"   • Reference: {ref}")
                if data.get('description'):
                    self.message_text.append(f"   • Description: {data['description']}")
                self.message_text.append("")
        else:
            self.message_text.append("No material lines to save.")

# =======================================================================================================================================
    def open_new_measurement_dialog(self):
        if not hasattr(self, 'current_worksheet_name') or not self.current_worksheet_name:
            QMessageBox.warning(self, "No Active Worksheet",
                                "Please create or open a worksheet first before creating a design layer.")
            return

        dialog = MeasurementNewDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            data = dialog.get_data()

            layer_name = data['layer_name']

            selected_point_cloud_file = data['point_cloud_file']  # This could be None if "No file selected"
            # Determine which point cloud file to use
            if selected_point_cloud_file and selected_point_cloud_file != "No file selected":
                # User selected a specific file from the dropdown
                point_cloud_file = selected_point_cloud_file
                should_load_pcd = True  # Flag to indicate we should load this file
            else:
                # User didn't select a file, keep the currently loaded file (if any)
                point_cloud_file = getattr(self, 'loaded_file_path', None)
                should_load_pcd = False  # Don't load anything new

            self.current_measurement_layer = layer_name
            
            # Store reference data in class attributes
            self.use_reference_layer = data['use_reference']
            if self.use_reference_layer:
                self.reference_design_layer = data['design_layer']
                self.reference_design_path = data['design_layer_file']
                self.reference_line_type = data['reference_line']
            else:
                self.reference_design_layer = None
                self.reference_design_path = None
                self.reference_line_type = None
            
            # Build path inside current worksheet
            base_measurement_path = os.path.join(self.WORKSHEETS_BASE_DIR, self.current_worksheet_name, "measurements")
            layer_folder = os.path.join(base_measurement_path, layer_name)

            # Create folder
            os.makedirs(layer_folder, exist_ok=True)

            # Prepare full config data
            full_config = {
                "layer_name": layer_name,
                "project_name": getattr(self, 'current_project_name', 'None'),
                "worksheet_name": self.current_worksheet_name,
                "created_by": self.current_user,
                "created_at": datetime.now().isoformat(),
                "point_cloud_file": point_cloud_file,
                "use_reference": data['use_reference'],
                "reference_line_type": data['reference_line'],
                "reference_design_layer": data['design_layer'],
                "reference_design_path": data['design_layer_file']
            }

            # Save config file
            config_file_path = os.path.join(layer_folder, "measurement_layer_config.txt")
            try:
                with open(config_file_path, 'w', encoding='utf-8') as f:
                    json.dump(full_config, f, indent=4)

                self.message_text.append(f"New Measurement layer created: {layer_name}")
                
                if data['use_reference']:
                    self.message_text.append(f" → Reference: {data['reference_line']} from {data['design_layer']}")

            except Exception as e:
                QMessageBox.critical(self, "Save Failed", f"Could not save measurement layer config:\n{str(e)}")
                return
            
            # Update UI
            self.right_section.setMinimumHeight(900)

            if hasattr(self, 'main_measurement_section'):
                self.main_measurement_section.setVisible(True)
            
            if hasattr(self, 'checkboxes'):
                if self.checkboxes.isVisible():
                    self.checkboxes.setVisible(False)
            
            if hasattr(self, 'three_D_frame'):
                self.three_D_frame.setVisible(True)

            if hasattr(self, 'mode_banner'):
                self.mode_banner.setText("MEASUREMENT MODE")
                self.mode_banner.setStyleSheet("""
                    QLabel {
                        font-weight: bold;
                        color: #4A148C;
                        background-color: #E1BEE7;
                        padding: 5px;
                        border-radius: 5px;
                    }
                """)
                self.mode_banner.setVisible(True)

            if layer_name:
                self.add_layer_to_panel(layer_name, '3D')   

            QMessageBox.information(self, "Success",
                                f"Measurement layer '{layer_name}' created successfully!")
    
            # Load the point cloud only if user selected a specific file
            if should_load_pcd and point_cloud_file:
                self.load_point_cloud_from_path(point_cloud_file) 

    def open_existing_measurement_layers(self):
        if not hasattr(self, 'current_worksheet_name') or not self.current_worksheet_name:
            QMessageBox.warning(self, "No Active Worksheet",
                                "Please create or open a worksheet first before creating a design layer.")
            return
        
        self.right_section.setMinimumHeight(900)
        self.main_measurement_section.setVisible(False)
        # Show the measurement layers checkboxes in left panel
        self.checkboxes.setVisible(True)

        self.three_D_frame.setVisible(True)


# ========================= Merger layer dialog open ===================================
# =======================================================================================================================================
    def show_merger_config_dialog(self):
        if not hasattr(self, 'current_worksheet_name') or not self.current_worksheet_name:
            QMessageBox.warning(self, "No Active Worksheet",
                                "Please create or open a worksheet first before creating a design layer.")
            return
        
        if hasattr(self, 'two_D_frame'):
            self.two_D_frame.setVisible(False)
            
        # Hide measurement layers when switching to design
        if self.checkboxes.isVisible():
            self.checkboxes.setVisible(False)

        if self.main_measurement_section.isVisible():
            self.main_measurement_section.setVisible(False)

        # Construct the worksheet_root path
        worksheet_root = os.path.join(self.WORKSHEETS_BASE_DIR, self.current_worksheet_name)

        dialog = MergerLayerConfigDialog(parent=self, worksheet_root=worksheet_root)
        if dialog.exec_() == QDialog.Accepted:
            config = dialog.get_configuration()

            layer_name = config['layer_name']
            point_cloud_file = self.loaded_file_path

            # Build path inside current worksheet
            base_measurement_path = os.path.join(self.WORKSHEETS_BASE_DIR, self.current_worksheet_name, "merger")
            layer_folder = os.path.join(base_measurement_path, layer_name)
            
            # Create folder
            os.makedirs(layer_folder, exist_ok=True)

            # Prepare full config data
            full_config = {
                "layer_name": layer_name,
                "project_name": getattr(self, 'current_project_name', 'None'),
                "worksheet_name": self.current_worksheet_name,
                "created_by": self.current_user,
                "created_at": datetime.now().isoformat(),
                "point_cloud_file": point_cloud_file,
                "dimension": "3D"
            }

            # Save config file
            config_file_path = os.path.join(layer_folder, "merger_layer_config.txt")
            try:
                with open(config_file_path, 'w', encoding='utf-8') as f:
                    json.dump(full_config, f, indent=4)

                self.message_text.append(f"New Merger layer created: {layer_name}")

            except Exception as e:
                QMessageBox.critical(self, "Save Failed", f"Could not save merger layer config:\n{str(e)}")
                return
            
            # Save the configuration data as JSON file
            try:
                # Create JSON filename: layer_name_merger.json
                json_filename = f"{layer_name}_merger.json"
                json_file_path = os.path.join(layer_folder, json_filename)
                
                # Create JSON data structure
                json_data = {
                    "merger_layer_name": layer_name,
                    "created_at": datetime.now().isoformat(),
                    "worksheet_root": worksheet_root,
                    "designs_path": os.path.join(worksheet_root, "designs"),
                    "merger_points": []
                }
                
                # Add verified world coordinates from dialog if available
                if hasattr(dialog, 'verified_coordinates'):
                    json_data["verified_coordinates"] = dialog.verified_coordinates
                
                # Add merger points configuration
                for point in config['merger_points']:
                    merger_point_data = {
                        "point_number": point['point_number'],
                        "primary_layer": point['primary_layer'],
                        "primary_layer_path": point['primary_layer_path'],
                        "primary_json_path": point.get('primary_json_path', ''),
                        "layers": []
                    }
                    
                    # Add layer information
                    for layer in point['layers']:
                        layer_data = {
                            "layer_number": layer['layer_number'],
                            "selected_layer": layer['selected_layer'],
                            "layer_path": layer['layer_path'],
                            "json_path": layer.get('json_path', ''),
                            "chainage": layer['chainage']
                        }
                        
                        # Check if this layer has verified world coordinates
                        if hasattr(dialog, 'verified_coordinates'):
                            key = f"point_{point['point_number']}_layer_{layer['layer_number']}"
                            if key in dialog.verified_coordinates:
                                layer_data["world_coordinates"] = dialog.verified_coordinates[key]["world_coordinates"]
                                layer_data["verification_status"] = "verified"
                            else:
                                layer_data["verification_status"] = "not_verified"
                        else:
                            layer_data["verification_status"] = "not_verified"
                        
                        merger_point_data["layers"].append(layer_data)
                    
                    json_data["merger_points"].append(merger_point_data)
                
                # Save JSON file
                with open(json_file_path, 'w', encoding='utf-8') as f:
                    json.dump(json_data, f, indent=4)
                
                self.message_text.append(f"Merger configuration saved as JSON: {json_filename}")
                
            except Exception as e:
                QMessageBox.warning(self, "JSON Save Warning", 
                                f"Could not save JSON configuration file:\n{str(e)}")

            self.bottom_section.setVisible(False)
            self.herarchy_section.setVisible(False)

            self.right_section.setMinimumHeight(900)

            if hasattr(self, 'three_D_frame'):
                    self.three_D_frame.setVisible(True)

            if hasattr(self, 'mode_banner'):
                self.mode_banner.setText("MERGER MODE")
                self.mode_banner.setStyleSheet("""
                    QLabel {
                        font-weight: bold;
                        color: #4A148C;
                        background-color: #E1BEE7;
                        padding: 5px;
                        border-radius: 5px;
                    }
                """)
                self.mode_banner.setVisible(True)

            if layer_name:
                self.add_layer_to_panel(layer_name, '3D')

            QMessageBox.information(self, "Success",
                                            f"Merger layer '{layer_name}' created successfully!") 
            
# =======================================================================================================================================
    def open_existing_worksheet(self):

        self.reset_all()
        dialog = ExistingWorksheetDialog(self)
        if dialog.exec_() != QDialog.Accepted:
            return

        data = dialog.get_selected_data()
        if not data:
            QMessageBox.warning(self, "Invalid Selection", "No valid worksheet/layer selected.")
            return

        config = data["worksheet_data"]
        worksheet_name = config.get("worksheet_name")
        layer_name = data["layer_name"]
        subfolder_type = data["subfolder_type"]  # "designs", "construction", or "merger"
        full_layer_path = data["full_layer_path"]

        # Worksheet root folder
        worksheet_root = os.path.dirname(os.path.dirname(full_layer_path))

        if not os.path.exists(full_layer_path):
            QMessageBox.critical(self, "Path Error", f"Layer folder not found:\n{full_layer_path}")
            return

        # Update state
        self.current_worksheet_name = worksheet_name
        self.current_project_name = config.get("project_name")
        self.current_worksheet_data = config
        self.current_layer_name = layer_name
        self.current_subfolder_type = subfolder_type

        self.display_current_worksheet(config)

        dimension = config.get("dimension")
        category = config.get("worksheet_category", "None")

        self.three_D_frame.setVisible(dimension == "3D")
        self.two_D_frame.setVisible(dimension == "2D")

        # self.reset_all()
        self.clear_baseline_planes()
        self.clear_reference_lines()          # Clear previous 2D lines
        self.clear_reference_actors()         # Clear previous 3D actors

        # Load layer config file (different name for construction vs design)
        config_filename = "Construction_Layer_config.txt" if subfolder_type == "construction" else "layer_config.txt"
        layer_config_path = os.path.join(full_layer_path, config_filename)
        layer_config = {}
        if os.path.exists(layer_config_path):
            try:
                with open(layer_config_path, 'r', encoding='utf-8') as f:
                    layer_config = json.load(f)
            except Exception as e:
                self.message_text.append(f"ERROR loading {config_filename}: {str(e)}")

        # Referenced design layer (for construction layers)
        referenced_design_layer = layer_config.get("reference_layer_2d") if subfolder_type == "construction" else None
        design_layer_path = None
        if referenced_design_layer:
            design_layer_path = os.path.join(worksheet_root, "designs", referenced_design_layer)

        zero_loaded = False
        design_points_loaded = False
        loaded_baselines = {}  # For 3D planes
        dotted_line_drawn = False

        # Load point cloud if linked
        pc_file = config.get("point_cloud_file")
        if pc_file and os.path.exists(pc_file):
            self.load_point_cloud_from_path(pc_file)

        # ===============================================================
        # DIFFERENT BEHAVIOR BASED ON SUBFOLDER TYPE
        # ===============================================================
        
        if subfolder_type == "designs":
            self.add_material_line_button.setVisible(False)

            self.right_section.setMinimumHeight(1200)

            # === DESIGN LAYER MODE ===
            # Load ALL baselines from this design layer
            loaded_baselines = self.load_all_baselines_from_layer(full_layer_path)

            # Load ALL JSON points into 3D point cloud
            design_points_loaded = self.load_json_files_to_3d_pointcloud(full_layer_path)

            # Draw ALL baselines on 2D graph (solid lines)
            for ltype in loaded_baselines.keys():
                self.redraw_baseline_on_graph(ltype, style="solid")

            # Load zero line from this design layer
            zero_loaded = self.load_zero_line_from_layer(full_layer_path)

            self.message_text.append(f"Design layer mode: Loaded {len(loaded_baselines)} baselines (solid lines)")

        elif subfolder_type == "construction":
            # === CONSTRUCTION LAYER MODE ===
            # Load only the REFERENCED baseline (dotted line)

            self.right_section.setMinimumHeight(1200)

            referenced_baseline = layer_config.get("base_lines_reference")

            if referenced_baseline:
                # Try construction layer first
                baseline_path = os.path.join(full_layer_path, referenced_baseline)
                source = "construction layer"

                # Fallback to design layer if not found
                if not os.path.exists(baseline_path) and design_layer_path:
                    baseline_path = os.path.join(design_layer_path, referenced_baseline)
                    source = "design layer"

                if os.path.exists(baseline_path):
                    try:
                        with open(baseline_path, 'r', encoding='utf-8') as f:
                            data = json.load(f)
                        ltype = "construction"  # default type for reference
                        polylines_2d = []
                        for poly in data.get("polylines", []):
                            poly_2d = [(pt["chainage_m"], pt["relative_elevation_m"]) for pt in poly["points"]]
                            if len(poly_2d) >= 2:
                                polylines_2d.append(poly_2d)
                        self.line_types[ltype]['polylines'] = polylines_2d
                        self.redraw_baseline_on_graph(ltype, style="dotted")
                        dotted_line_drawn = True
                    except Exception as e:
                        self.message_text.append(f"Failed to load reference baseline: {e}")
                else:
                    self.message_text.append(f"Referenced baseline not found: {referenced_baseline}")

            # Load 3D planes ONLY from the referenced design layer (if any)
            if design_layer_path and os.path.exists(design_layer_path):
                loaded_baselines = self.load_all_baselines_from_layer(design_layer_path)
                design_points_loaded = self.load_json_files_to_3d_pointcloud(design_layer_path)
                self.message_text.append(f"Construction mode: Loaded {len(loaded_baselines)} baselines from referenced design layer {referenced_design_layer}")

            # Load zero line: prefer design layer, then current construction layer
            if design_layer_path and os.path.exists(design_layer_path):
                zero_loaded = self.load_zero_line_from_layer(design_layer_path)
            if not zero_loaded:
                zero_loaded = self.load_zero_line_from_layer(full_layer_path)

        elif subfolder_type == "merger":
            self.right_section.setMinimumHeight(1200)

            # === MERGER LAYER MODE ===
            # STEP 1: Find and load the merger JSON file
            merger_json_files = [f for f in os.listdir(full_layer_path) if f.endswith('.json')]
            if not merger_json_files:
                QMessageBox.warning(self, "No Merger JSON", f"No JSON file found in merger layer: {full_layer_path}")
                return
            
            merger_json_path = os.path.join(full_layer_path, merger_json_files[0])
            
            try:
                with open(merger_json_path, 'r', encoding='utf-8') as f:
                    merger_data = json.load(f)

                # Call the separate function to create hierarchy
                self.create_merger_hierarchy(merger_data, full_layer_path)

                # Call the new function to process merger data
                loaded_baselines, zero_loaded = self.process_merger_data(merger_data, full_layer_path)
                
            except Exception as e:
                QMessageBox.critical(self, "Merger JSON Error", f"Failed to load merger JSON: {str(e)}")
                import traceback
                traceback.print_exc()
                return

        # ===============================================================
        # Generate 3D planes if we have baselines and zero line
        # ===============================================================
        if self.zero_line_set and loaded_baselines:
            self.generate_3d_planes_from_baselines(loaded_baselines)

        # ===============================================================
        # UI setup
        # ===============================================================
        self.show_graph_section(category)

        is_construction_layer = (subfolder_type == "construction")
        if is_construction_layer:
            self.switch_to_construction_mode()
            self.zero_container.setVisible(False)
            self.scale_section.setVisible(True)
            self.message_text.append("Construction mode activated")
        else:
            self.surface_container.setVisible(category == "Road")
            self.construction_container.setVisible(category == "Road")
            self.road_surface_container.setVisible(category == "Road")
            self.zero_container.setVisible(category in ["Road", "Bridge"])
            self.preview_button.setVisible(True)
            self.elivation_angle_button.setVisible(True)
            self.threed_map_button.setVisible(True)
            self.save_button.setVisible(True)

        if subfolder_type == "merger":
            self.two_D_frame.setVisible(False)
            self.three_D_frame.setVisible(True)

            dimension = "3D"
            self.bottom_section.setVisible(False)
            self.herarchy_section.setVisible(True)

            if hasattr(self, 'mode_banner'):
                self.mode_banner.setText("MERGER MODE")
                self.mode_banner.setStyleSheet("""
                    QLabel {
                        font-weight: bold;
                    }
                """)
                self.mode_banner.setVisible(True)

        self.add_layer_to_panel(layer_name, dimension)

        # Final feedback
        self.message_text.append(f"Successfully opened worksheet:")
        self.message_text.append(f"   → Worksheet: {worksheet_name}")
        self.message_text.append(f"   → Section: {subfolder_type}")
        self.message_text.append(f"   → Layer: {layer_name}")
        
        if subfolder_type == "merger":
            self.message_text.append(f"   → 2D graph: Disabled in merger mode")
            self.message_text.append(f"   → 3D planes: {len(loaded_baselines)} baseline types from merger points")
        else:
            self.message_text.append(f"   → 2D graph: {'All baselines (solid)' if subfolder_type == 'designs' else 'Reference baseline (dotted)' if dotted_line_drawn else 'No reference baseline'}")
            self.message_text.append(f"   → 3D planes: {len(loaded_baselines)} baselines {'from design layer' if subfolder_type == 'construction' else 'from this layer'}")
        
        if referenced_design_layer:
            self.message_text.append(f"   → Design ref: {referenced_design_layer} {'(loaded)' if design_points_loaded else '(not found)'}")
        if subfolder_type == "construction" and referenced_baseline:
            self.message_text.append(f"   → Baseline ref: {referenced_baseline} (dotted)")
        self.message_text.append(f"   → Zero line: {'Loaded' if zero_loaded else 'Not found'}")

        QMessageBox.information(self, "Worksheet Opened",
                                f"<b>{worksheet_name}</b><br><br>"
                                f"Layer: <i>{layer_name}</i> ({subfolder_type})<br>"
                                f"Mode: {'Construction' if is_construction_layer else 'Design' if subfolder_type == 'designs' else 'Merger'}<br>"
                                f"3D Planes: {len(loaded_baselines)} baselines loaded<br>"
                                f"Zero Line: {'Yes' if zero_loaded else 'No'}")

        self.canvas.draw_idle()
        if hasattr(self, 'vtk_widget'):
            self.vtk_widget.GetRenderWindow().Render()
    
# ================= Merger layer & its hierarchy creation =======================
    def create_merger_hierarchy(self, merger_data, full_layer_path):
        """
        Create the hierarchy section for merger layers.
        """
        # ============================================
        # CREATE MERGER SECTIONS IN HIERARCHY SECTION
        # ============================================
        if not hasattr(self, 'herarchy_section'):
            return
    
        # METHOD 1: Complete replacement - create a new widget entirely
        # This is the most reliable approach
        try:
            # Get the parent of the current herarchy_section
            parent_widget = self.herarchy_section.parent()
            
            # Create a completely new QFrame
            new_herarchy_section = QFrame()
            new_herarchy_section.setObjectName("herarchy_section")
            new_herarchy_section.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
            
            # Replace the old herarchy_section in its parent layout
            if parent_widget and parent_widget.layout():
                # Find the position of the old widget in the layout
                for i in range(parent_widget.layout().count()):
                    item = parent_widget.layout().itemAt(i)
                    if item and item.widget() == self.herarchy_section:
                        # Replace it
                        parent_widget.layout().removeWidget(self.herarchy_section)
                        parent_widget.layout().insertWidget(i, new_herarchy_section)
                        break
            
            # Delete the old widget
            self.herarchy_section.deleteLater()
            
            # Update the reference
            self.herarchy_section = new_herarchy_section
            
        except Exception as e:
            print(f"Error replacing herarchy_section: {e}")
            # Fall back to clearing method
            self.clear_hierarchy_section_completely()

        # Now create the content in the new/cleared section
        self.build_merger_hierarchy_content(merger_data, full_layer_path)

    def clear_hierarchy_section_completely(self):
        """Completely clear the hierarchy section"""
        if not hasattr(self, 'herarchy_section'):
            return
        
        # Remove all child widgets
        for child in self.herarchy_section.findChildren(QWidget):
            child.deleteLater()
        
        # Clear any existing layout
        old_layout = self.herarchy_section.layout()
        if old_layout:
            # Use a helper to properly clear the layout
            self.clear_layout_completely(old_layout)

    def clear_layout_completely(self, layout):
        """Completely clear a layout and all its widgets"""
        if layout is None:
            return
        
        # Disconnect signals first
        try:
            layout.disconnect()
        except:
            pass
        
        # Remove all items
        while layout.count():
            item = layout.takeAt(0)
            
            if item.widget():
                # Widget item
                widget = item.widget()
                widget.hide()
                widget.deleteLater()
            elif item.layout():
                # Sublayout
                sublayout = item.layout()
                self.clear_layout_completely(sublayout)
                sublayout.deleteLater()
            elif item.spacerItem():
                # Spacer item
                layout.removeItem(item)

    def build_merger_hierarchy_content(self, merger_data, full_layer_path):
        """Build the actual content of the merger hierarchy"""
        
        self.herarchy_section.setStyleSheet("""
            QFrame#herarchy_section {
                border: 2px solid #7B1FA2;
                border-radius: 8px;
                background-color: #8FBFEF;
                padding: 5px;
            }
        """)
        
        # Set frame style for better appearance
        self.herarchy_section.setFrameStyle(QFrame.Box | QFrame.Raised)
        self.herarchy_section.setMinimumHeight(350)
        self.herarchy_section.setLineWidth(2)

        # Create a new layout (or reuse existing)
        if self.herarchy_section.layout():
            layout = self.herarchy_section.layout()
            self.clear_layout_completely(layout)
        else:
            layout = QVBoxLayout(self.herarchy_section)
        
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(0)
        
        # Create a title for the hierarchy section
        title_label = QLabel("Merger Hierarchy")
        title_label.setStyleSheet("""
            QLabel {
                font-weight: bold;
                font-size: 16px;
                color: #4A148C;
                padding: 5px;
                background-color: #E3F2FD;
                border-radius: 5px;
                border: 1px solid #90CAF9;
            }
        """)
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setFixedHeight(40)
        layout.addWidget(title_label)
        
        # Create a scroll area for merger points
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setStyleSheet("""
            QScrollArea {
                border: none;
                background-color: transparent;
            }
            QScrollBar:horizontal {
                border: none;
                background: #F0F0F0;
                height: 10px;
                border-radius: 5px;
            }
            QScrollBar::handle:horizontal {
                background: #BDBDBD;
                border-radius: 5px;
                min-width: 30px;
            }
            QScrollBar::handle:horizontal:hover {
                background: #9E9E9E;
            }
        """)
        
        # Create container widget for merger points
        merger_points_container = QWidget()
        merger_points_layout = QHBoxLayout(merger_points_container)
        merger_points_layout.setSpacing(10)
        merger_points_layout.setContentsMargins(0, 0, 0, 0)
        merger_points_layout.setAlignment(Qt.AlignTop)

        # Add a merger section for each merger point
        for merger_point in merger_data.get("merger_points", []):
            point_number = merger_point.get("point_number", "Unknown")
            
            # Create frame for this merger point
            merger_point_frame = QFrame()
            merger_point_frame.setFrameStyle(QFrame.Box | QFrame.Raised)
            merger_point_frame.setStyleSheet("""
                QFrame {
                    border: 2px solid #7B1FA2;
                    border-radius: 8px;
                    background-color: #F3E5F5;
                    padding: 0px;
                    min-width: 300px;
                    max-width: 320px;
                }
            """)
            
            # Create layout for this merger point
            point_layout = QVBoxLayout(merger_point_frame)
            point_layout.setContentsMargins(5, 5, 5, 5)
            point_layout.setSpacing(10)
            
            # Create title for this merger point
            point_title = QLabel(f"Merger Point {point_number}")
            point_title.setFixedHeight(30)
            point_title.setStyleSheet("""
                QLabel {
                    font-weight: bold;
                    font-size: 14px;
                    color: #7B1FA2;
                    background-color: #E1BEE7;
                    border-radius: 4px;
                    text-align: center;
                    min-width: 260px;
                }
            """)
            point_title.setAlignment(Qt.AlignCenter)
            point_layout.addWidget(point_title)
            
            # Get selected layers for this merger point
            layers = merger_point.get("layers", [])

            # Create a scroll area for layers inside each merger point
            layers_scroll = QScrollArea()
            layers_scroll.setWidgetResizable(True)
            layers_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            layers_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
            layers_scroll.setStyleSheet("""
                QScrollArea {
                    border: none;
                    background-color: transparent;
                }
                QScrollBar:vertical {
                    border: none;
                    background: #F0F0F0;
                    width: 8px;
                    border-radius: 4px;
                }
                QScrollBar::handle:vertical {
                    background: #BDBDBD;
                    border-radius: 4px;
                    min-height: 30px;
                }
            """)
            
            # Create container for layers
            layers_container = QWidget()
            layers_layout = QVBoxLayout(layers_container)
            layers_layout.setSpacing(8)
            layers_layout.setContentsMargins(0, 0, 0, 0)
            
            # Add collapsible layer sections
            for layer in layers:
                layer_name = layer.get("selected_layer", "Unnamed Layer")
                json_path = layer.get("json_path")
                merger_layer_path = layer.get("layer_path")
                
                # Create collapsible container for each layer
                layer_container = QWidget()
                layer_container.setStyleSheet("""
                    QWidget {
                        background-color: transparent;
                        min-width: 270px;
                        max-width: 280px;
                    }
                """)
                
                layer_main_layout = QVBoxLayout(layer_container)
                layer_main_layout.setSpacing(0)
                layer_main_layout.setContentsMargins(0, 0, 0, 0)
                
                # Create expandable header
                header_widget = QWidget()
                header_widget.setCursor(Qt.PointingHandCursor)
                header_widget.setStyleSheet("""
                    QWidget {
                        background-color: #EDE7F6;
                        border-radius: 4px;
                        padding: 5px;
                    }
                    QWidget:hover {
                        background-color: #D1C4E9;
                    }
                """)
                
                header_layout = QHBoxLayout(header_widget)
                header_layout.setContentsMargins(0, 0, 0, 0)
                header_layout.setSpacing(5)
                
                # Chevron icon (initially pointing right)
                chevron_label = QLabel("▶")
                chevron_label.setStyleSheet("""
                    QLabel {
                        color: #7B1FA2;
                        font-size: 20px;
                        font-weight: bold;
                        min-width: 30px;
                    }
                """)
                chevron_label.setProperty("expanded", False)
                
                # Layer name
                layer_name_label = QLabel(layer_name)
                layer_name_label.setStyleSheet("""
                    QLabel {
                        color: #5D4037;
                        font-size: 13px;
                        font-weight: bold;
                    }
                """)
                
                header_layout.addWidget(chevron_label)
                header_layout.addWidget(layer_name_label)
                
                # Content area (initially hidden)
                content_widget = QWidget()
                content_widget.setVisible(False)
                content_widget.setStyleSheet("""
                    QWidget {
                        background-color: #FAFAFA;
                        border-left: 2px solid #D1C4E9;
                        margin-left: 10px;
                        margin-top: 5px;
                    }
                """)
                
                content_layout = QVBoxLayout(content_widget)
                content_layout.setSpacing(5)
                content_layout.setContentsMargins(15, 10, 5, 10)
                
                # Create checkboxes for sub-layers
                sub_layer_checkbox1 = self.create_simple_layer_item("Road Surface Line", merger_layer_path)
                sub_layer_checkbox2 = self.create_simple_layer_item("Surface Line", merger_layer_path)
                sub_layer_checkbox3 = self.create_simple_layer_item("Construction Line", merger_layer_path)
                
                content_layout.addWidget(sub_layer_checkbox1)
                content_layout.addWidget(sub_layer_checkbox2)
                content_layout.addWidget(sub_layer_checkbox3)
                content_layout.addStretch()
                
                def create_toggle_func(merger_layer_path, chevron, content, container):
                    def toggle_expansion():
                        is_expanded = not content.isVisible()
                        content.setVisible(is_expanded)
                        if is_expanded:
                            chevron.setText("▼")
                            chevron.setProperty("expanded", is_expanded)
                            self.right_section.setMinimumHeight(1400)
                            self.bottom_section.setVisible(True)
                            # Load zero line from this design layer
                            zero_loaded = self.load_zero_line_from_layer(merger_layer_path)
                        else:
                            chevron.setText("▶")
                            chevron.setProperty("expanded", is_expanded)
                            self.right_section.setMinimumHeight(1200)
                            self.bottom_section.setVisible(False)

                        # Adjust size
                        container.adjustSize()
                        layers_container.adjustSize()
                    return toggle_expansion
                
                # Create the toggle function with the current layer path
                toggle_func = create_toggle_func(
                    merger_layer_path=merger_layer_path,
                    chevron=chevron_label,
                    content=content_widget,
                    container=layer_container
                )
                
                # Connect header click to toggle
                header_widget.mousePressEvent = lambda e, func=toggle_func: func()
                
                # Add to main layout
                layer_main_layout.addWidget(header_widget)
                layer_main_layout.addWidget(content_widget)
                
                layers_layout.addWidget(layer_container)
            
            # If no layers found, add a placeholder
            if not layers:
                no_layers_label = QLabel("No layers available")
                no_layers_label.setStyleSheet("color: #757575; font-style: italic;")
                no_layers_label.setAlignment(Qt.AlignCenter)
                layers_layout.addWidget(no_layers_label)

            layers_layout.addStretch()
        
            # Set the layers container to scroll area
            layers_scroll.setWidget(layers_container)
            
            # Add the scroll area to the point layout
            point_layout.addWidget(layers_scroll)
            
            # Add the merger point frame to container
            merger_points_layout.addWidget(merger_point_frame)
        
        # Set the container widget to scroll area
        scroll_area.setWidget(merger_points_container)
        
        # Add scroll area to hierarchy section layout
        layout.addWidget(scroll_area)
        
        # Make hierarchy section visible
        self.herarchy_section.setVisible(True)

    def process_merger_data(self, merger_data, full_layer_path):
        """
        Process merger data to load baselines, add markers, and load zero lines.
        
        Args:
            merger_data: Dictionary containing merger data
            full_layer_path: Path to the merger layer folder
            
        Returns:
            tuple: (loaded_baselines dictionary, zero_loaded boolean)
        """
        # Collect all json_paths from all merger points
        all_json_paths = []
        json_paths_by_point = {}  # Store json paths grouped by point number
        # Store world coordinates for each merger point to compute averages
        merger_point_world_coords = {}  # key: point_number, value: list of (x, y, z) tuples
        
        for merger_point in merger_data.get("merger_points", []):
            point_number = merger_point.get("point_number")
            json_paths_by_point[point_number] = []
            merger_point_world_coords[point_number] = []  # Initialize empty list for coordinates
            
            # Add primary layer JSON path
            primary_json_path = merger_point.get("primary_json_path")
            if primary_json_path and os.path.exists(primary_json_path):
                if primary_json_path not in all_json_paths:
                    all_json_paths.append(primary_json_path)
                json_paths_by_point[point_number].append(primary_json_path)
            
            # Add all layers' JSON paths
            for layer in merger_point.get("layers", []):
                json_path = layer.get("json_path")
                if json_path and os.path.exists(json_path) and json_path not in all_json_paths:
                    all_json_paths.append(json_path)
                json_paths_by_point[point_number].append(json_path)
        
        # Debug: Print JSON paths
        self.message_text.append(f"Found {len(all_json_paths)} JSON paths to process")
        
        # STEP 2: Process each merger point and load its layers
        loaded_baselines = {}
        
        for point_number, json_paths in json_paths_by_point.items():
            self.message_text.append(f"Processing Merger Point {point_number} with {len(json_paths)} layers")
            
            # Process each JSON path for this point
            for json_path in json_paths:
                if not os.path.exists(json_path):
                    self.message_text.append(f"  Warning: JSON file not found: {json_path}")
                    continue
                
                try:
                    # Load the baseline JSON file
                    with open(json_path, 'r', encoding='utf-8') as f:
                        baseline_data = json.load(f)
                    
                    # Debug: Check if world_coordinates exist
                    has_world_coords = False
                    for polyline in baseline_data.get("polylines", []):
                        points = polyline.get("points", [])
                        for point in points:
                            if "world_coordinates" in point:
                                has_world_coords = True
                                break
                        if has_world_coords:
                            break
                    
                    if has_world_coords:
                        self.message_text.append(f"  ✓ Found world_coordinates in {os.path.basename(json_path)}")
                    else:
                        self.message_text.append(f"  ✗ No world_coordinates found in {os.path.basename(json_path)}")
                    
                    # Get the baseline_key from JSON - this should match keys in self.plane_colors
                    baseline_key = baseline_data.get("baseline_key", "unknown")
                    width_meters = baseline_data.get("width_meters", 10.0)
                    
                    # Extract layer name from path
                    layer_folder = os.path.basename(os.path.dirname(json_path))
                    json_filename = os.path.basename(json_path)
                    
                    # Use baseline_key directly as the ltype
                    # This should match keys in self.plane_colors (e.g., "road_surface", "deck_line", etc.)
                    ltype = baseline_key
                    
                    # If baseline_key is not in plane_colors, use a default
                    if ltype not in self.plane_colors:
                        # Try to map common baseline types
                        if "road_surface" in baseline_key.lower():
                            ltype = "road_surface"
                        elif "deck" in baseline_key.lower():
                            ltype = "deck_line"
                        else:
                            ltype = "surface"  # default

                    if ltype not in loaded_baselines:
                        loaded_baselines[ltype] = {
                            "polylines": [],
                            "width_meters": width_meters
                        }
                    
                    # Add polylines from this JSON and collect world coordinates
                    for polyline in baseline_data.get("polylines", []):
                        # Each point should already have world_coordinates
                        points = polyline.get("points", [])
                        if points:
                            # Add to loaded baselines
                            loaded_baselines[ltype]["polylines"].append({
                                "points": points,
                                "start_chainage_m": polyline.get("start_chainage_m", 0),
                                "end_chainage_m": polyline.get("end_chainage_m", 0)
                            })
                            
                            # Collect world coordinates for average calculation
                            for point in points:
                                world_coords = point.get("world_coordinates")
                                if world_coords and len(world_coords) == 3:
                                    merger_point_world_coords[point_number].append(world_coords)
                    
                    self.message_text.append(f"  ✓ Loaded: {layer_folder}/{json_filename} - {baseline_key} -> using '{ltype}' color")
                    
                except Exception as e:
                    self.message_text.append(f"  ✗ Error loading {json_path}: {str(e)}")
                    import traceback
                    traceback.print_exc()
        
        # Debug: Print collected coordinates
        for point_number, coords_list in merger_point_world_coords.items():
            self.message_text.append(f"Merger Point {point_number} has {len(coords_list)} world coordinates")
        
        # STEP 3: Calculate average world coordinates for each merger point and add markers
        for point_number, coords_list in merger_point_world_coords.items():
            if coords_list:
                # Calculate average coordinates
                avg_x = sum(coord[0] for coord in coords_list) / len(coords_list)
                avg_y = sum(coord[1] for coord in coords_list) / len(coords_list)
                avg_z = sum(coord[2] for coord in coords_list) / len(coords_list)
                
                self.message_text.append(f"  Merger Point {point_number} average coordinates: ({avg_x:.2f}, {avg_y:.2f}, {avg_z:.2f})")
                
                # Add vertical marker at average coordinates
                marker_added = self.add_vertical_marker_at_point(avg_x, avg_y, avg_z, point_number)
                if marker_added:
                    self.message_text.append(f"  ✓ Added marker for Merger Point {point_number}")
                else:
                    self.message_text.append(f"  ✗ Failed to add marker for Merger Point {point_number}")
        
        self.message_text.append(f"Merger mode: Loaded {len(loaded_baselines)} baseline types from {len(all_json_paths)} JSON files")
        
        # STEP 4: Try to load zero line from the primary layer
        zero_loaded = False
        
        if merger_data.get("merger_points"):
            first_point = merger_data["merger_points"][0]
            primary_layer_path = first_point.get("primary_layer_path")
            if primary_layer_path:
                zero_loaded = self.load_zero_line_from_layer(primary_layer_path)
                if zero_loaded:
                    self.message_text.append(f"Zero line loaded from primary layer: {os.path.basename(primary_layer_path)}")
        
        # If zero line not loaded from primary, try any available layer
        if not zero_loaded and all_json_paths:
            for json_path in all_json_paths:
                layer_path = os.path.dirname(json_path)
                zero_loaded = self.load_zero_line_from_layer(layer_path)
                if zero_loaded:
                    self.message_text.append(f"Zero line loaded from: {os.path.basename(layer_path)}")
                    break
        
        return loaded_baselines, zero_loaded

    def create_simple_layer_item(self, name, path):
        """Create simple layer item with checkbox and pencil button"""
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)
        
        # Checkbox
        checkbox = QCheckBox(name)

        # Connect checkbox state change to toggle function
        def on_checkbox_state_changed(state):
            self.toggle_merger_baselines_checkbox(
                layer_name=name,
                layer_path=path,
                is_checked=state == Qt.Checked,
            )
        
        checkbox.stateChanged.connect(on_checkbox_state_changed)

        layout.addWidget(checkbox)
        
        return widget
    
    def toggle_merger_baselines_checkbox(self, layer_name, layer_path, is_checked):
        """Handle toggling of merger baseline checkboxes"""
        if is_checked:
            # Load baseline data from the layer
            loaded_baselines = self.load_specific_baselines_from_layer(layer_name, layer_path)
            
            if loaded_baselines:
                # Generate 3D planes for each baseline type
                for baseline_type, baseline_data in loaded_baselines.items():
                    # Add layer path to baseline data for identification
                    baseline_data["layer_path"] = layer_path
                    
                    # Generate 3D planes using the same logic as map_baselines_to_3d_planes_from_data
                    actors = self.generate_3d_planes_for_baseline(baseline_data, baseline_type)
                    
                    if actors:
                        # Store actors for later removal
                        self.merger_plane_actors[(layer_path, baseline_type)] = actors
                
                # Draw ALL baselines on 2D graph (solid lines)
                for ltype in loaded_baselines.keys():
                    self.redraw_baseline_on_graph(ltype, style="solid")
                    
                self.message_text.append(f"✓ Loaded and displayed '{layer_name}' with {len(loaded_baselines)} baseline types")
        else:
            # We need to determine which baseline type(s) to remove based on layer_name
            # Different layer names map to different baseline types
            baseline_types_to_remove = []
            
            # Map layer names to baseline types (reverse of what we use in loading)
            layer_to_baseline_map = {
                "Road Surface Line": "road_surface",
                "Surface Line": "surface", 
                "Construction Line": "construction"
            }
            
            baseline_type = layer_to_baseline_map.get(layer_name)
            
            if baseline_type:
                # Remove only the specific baseline type for this layer
                self.remove_specific_3d_planes(layer_path, baseline_type)
                
                # Also clear the 2D graph lines for this specific baseline type
                self.clear_specific_2d_baseline(baseline_type)
                
                self.message_text.append(f"✓ Removed 3D planes for '{layer_name}' (baseline type: {baseline_type})")
            else:
                # Fallback: remove all planes for this layer if we can't determine the type
                self.remove_all_planes_for_layer(layer_path, layer_name)
    
    def load_specific_baselines_from_layer(self, layer_name, layer_path):
        """Load specific baseline from a layer and return the data."""
        loaded = {}
        
        # Map layer names to filenames
        filename_map = {
            "Road Surface Line": "road_surface_baseline.json",
            "Surface Line": "surface_baseline.json",
            "Construction Line": "construction_baseline.json"
        }
        
        filename = filename_map.get(layer_name)
        if not filename:
            self.message_text.append(f"Unknown layer name: {layer_name}")
            return loaded
        
        key_map = {
            "surface_baseline.json": "surface",
            "construction_baseline.json": "construction",
            "road_surface_baseline.json": "road_surface",
            "deck_line_baseline.json": "deck_line",
            "projection_line_baseline.json": "projection_line",
            "material_baseline.json": "material",
        }
        
        filepath = os.path.join(layer_path, filename)
        if not os.path.exists(filepath):
            self.message_text.append(f"Baseline file not found: {filename}")
            return loaded
        
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            baseline_type = key_map.get(filename)
            if baseline_type:
                # Ensure we have the required structure
                if "polylines" not in data:
                    data["polylines"] = []
                
                # Ensure width_meters is present
                if "width_meters" not in data:
                    data["width_meters"] = 10.0  # Default width
                    
                # Validate that points have world_coordinates
                for polyline in data["polylines"]:
                    points = polyline.get("points", [])
                    for point in points:
                        if "world_coordinates" not in point:
                            # Try to create world coordinates from available data
                            chainage = point.get("chainage_m", 0)
                            elevation = point.get("relative_elevation_m", 0)
                            point["world_coordinates"] = [chainage, elevation, 0]
                
                loaded[baseline_type] = data
                
                # Store polylines for 2D graph (centerline visualization)
                polylines_2d = []
                for poly in data.get("polylines", []):
                    poly_2d = [(pt.get("chainage_m", 0), pt.get("relative_elevation_m", 0)) 
                            for pt in poly.get("points", [])]
                    if len(poly_2d) >= 2:
                        polylines_2d.append(poly_2d)
                
                self.line_types[baseline_type]['polylines'] = polylines_2d
                
            self.message_text.append(f"✓ Loaded baseline: {layer_name} -> {baseline_type}, width: {data.get('width_meters', 10.0)}m")
            
        except Exception as e:
            self.message_text.append(f"Error loading {filename}: {str(e)}")
            import traceback
            traceback.print_exc()
        
        return loaded
        
    def remove_specific_3d_planes(self, layer_path, baseline_type):
        """
        Remove 3D plane actors for a specific baseline type in a specific layer.
        
        Args:
            layer_path: Path to the layer folder
            baseline_type: Type of baseline to remove (e.g., "surface", "construction")
        """
        key = (layer_path, baseline_type)
        
        if key in self.merger_plane_actors:
            actors = self.merger_plane_actors[key]
            
            # Remove actors from renderer
            if hasattr(self, 'renderer'):
                for actor in actors:
                    self.renderer.RemoveActor(actor)
            
            # Remove from tracking dictionary
            del self.merger_plane_actors[key]
            
            # Update render window
            if hasattr(self, 'vtk_widget'):
                self.vtk_widget.GetRenderWindow().Render()
            
            self.message_text.append(f"Removed {len(actors)} plane actors for {baseline_type}")
        else:
            self.message_text.append(f"No planes found for {baseline_type} in layer {os.path.basename(layer_path)}")

    def clear_specific_2d_baseline(self, baseline_type):
        """
        Clear 2D graph lines for a specific baseline type.
        
        Args:
            baseline_type: Type of baseline to clear (e.g., "surface", "construction")
        """
        if baseline_type in self.line_types:
            # Clear polylines for this baseline type
            self.line_types[baseline_type]['polylines'] = []
            
            # Redraw the graph to remove the lines
            if hasattr(self, 'ax') and self.ax:
                # Find and remove the line artists for this baseline type
                artists_to_remove = []
                for artist in self.ax.lines + self.ax.collections:
                    if hasattr(artist, '_baseline_type') and artist._baseline_type == baseline_type:
                        artists_to_remove.append(artist)
                
                for artist in artists_to_remove:
                    artist.remove()
                
                self.canvas.draw_idle()

    def remove_all_planes_for_layer(self, layer_path, layer_name):
        """
        Fallback: Remove all 3D plane actors for a specific layer.
        Only used when we can't determine the specific baseline type.
        
        Args:
            layer_path: Path to the layer folder
            layer_name: Name of the layer (for logging)
        """
        actors_to_remove = []
        keys_to_delete = []
        
        # Find all actors for this layer
        for key, actors in self.merger_plane_actors.items():
            stored_layer_path, baseline_type = key
            if stored_layer_path == layer_path:
                actors_to_remove.extend(actors)
                keys_to_delete.append(key)
                
                # Also clear 2D graph for this baseline type
                self.clear_specific_2d_baseline(baseline_type)
        
        # Remove actors from renderer
        if hasattr(self, 'renderer'):
            for actor in actors_to_remove:
                self.renderer.RemoveActor(actor)
        
        # Remove from tracking dictionary
        for key in keys_to_delete:
            if key in self.merger_plane_actors:
                del self.merger_plane_actors[key]
        
        # Update render window
        if actors_to_remove and hasattr(self, 'vtk_widget'):
            self.vtk_widget.GetRenderWindow().Render()
        
        self.message_text.append(f"Removed all planes for layer '{layer_name}' ({len(actors_to_remove)} actors)")
        
    def generate_3d_planes_from_baselines(self, loaded_baselines):
        """
        Generate 3D planes from loaded baselines if zero line is set.
        
        Args:
            loaded_baselines: Dictionary containing loaded baseline data
            
        Returns:
            bool: True if planes were generated, False otherwise
        """
        if not self.zero_line_set:
            self.message_text.append("Cannot generate 3D planes: Zero line not set")
            return False
        
        if not loaded_baselines:
            self.message_text.append("Cannot generate 3D planes: No baselines loaded")
            return False
        
        # Determine width from loaded baselines or use default
        last_width = 10.0
        if loaded_baselines:
            first_key = list(loaded_baselines.keys())[0]
            baseline_data = loaded_baselines[first_key]
            last_width = baseline_data.get("width_meters", 10.0)
        
        self.last_plane_width = last_width
        
        # Call the function to create 3D planes
        success = self.map_baselines_to_3d_planes_from_data(loaded_baselines, last_width)
        
        if success:
            self.message_text.append(f"Generated 3D planes from {len(loaded_baselines)} baseline types")
            return True
        else:
            self.message_text.append("Failed to generate 3D planes")
            return False   

    def generate_3d_planes_for_baseline(self, baseline_data, baseline_type):
        """
        Generate 3D plane actors for a specific baseline type using vtkPlaneSource.
        This matches the existing map_baselines_to_3d_planes_from_data logic.
        
        Args:
            baseline_data: Dictionary containing baseline data with width
            baseline_type: Type of baseline (e.g., "road_surface", "surface", "construction")
            
        Returns:
            list: List of VTK actors created for this baseline
        """
        if not self.zero_line_set:
            self.message_text.append(f"Cannot generate 3D planes for {baseline_type}: Zero line not set")
            return []
        
        actors = []
        
        # Get color and opacity for this baseline type
        rgba = self.plane_colors.get(baseline_type, (0.5, 0.5, 0.5, 0.4))
        color_rgb = rgba[:3]
        opacity = rgba[3]
        
        # Get width from baseline data (default to 10.0 if not specified)
        width = baseline_data.get("width_meters", 10.0)
        half_width = width / 2.0
        
        # Get zero line information
        if not hasattr(self, 'zero_start_point') or not hasattr(self, 'zero_end_point'):
            self.message_text.append("Zero line points not available")
            return []
        
        zero_dir_vec = np.array(self.zero_end_point) - np.array(self.zero_start_point)
        
        # Process each polyline in the baseline data
        for polyline in baseline_data.get("polylines", []):
            points_3d = polyline.get("points", [])
            if len(points_3d) < 2:
                continue
            
            # Create plane for each segment
            for i in range(len(points_3d) - 1):
                pt1 = points_3d[i]
                pt2 = points_3d[i + 1]
                
                # Get world coordinates
                center1 = np.array(pt1.get("world_coordinates", [0, 0, 0]))
                center2 = np.array(pt2.get("world_coordinates", [0, 0, 0]))
                
                # Calculate segment direction and length
                seg_dir = center2 - center1
                seg_len = np.linalg.norm(seg_dir)
                if seg_len < 1e-6:
                    continue
                seg_unit = seg_dir / seg_len
                
                # Calculate perpendicular direction (same logic as map_baselines_to_3d_planes_from_data)
                horiz = np.array([seg_unit[0], seg_unit[1], 0.0])
                hlen = np.linalg.norm(horiz)
                
                if hlen < 1e-6:
                    # If segment is vertical, use zero line direction for perpendicular
                    zero_length = np.linalg.norm(zero_dir_vec)
                    if zero_length < 1e-6:
                        continue
                    zero_unit = zero_dir_vec / zero_length
                    perp = np.array([-zero_unit[1], zero_unit[0], 0.0])
                else:
                    horiz /= hlen
                    perp = np.array([-horiz[1], horiz[0], 0.0])
                
                perp_len = np.linalg.norm(perp)
                if perp_len > 0:
                    perp /= perp_len
                
                # Calculate the 4 corners of the plane
                c1 = center1 + perp * half_width
                c2 = center1 - perp * half_width
                c3 = center2 - perp * half_width
                c4 = center2 + perp * half_width
                
                # Create the plane using vtkPlaneSource (same as original)
                plane = vtk.vtkPlaneSource()
                plane.SetOrigin(c1[0], c1[1], c1[2])
                plane.SetPoint1(c4[0], c4[1], c4[2])
                plane.SetPoint2(c2[0], c2[1], c2[2])
                plane.SetXResolution(12)
                plane.SetYResolution(2)
                plane.Update()
                
                # Create mapper and actor
                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputConnection(plane.GetOutputPort())
                
                actor = vtk.vtkActor()
                actor.SetMapper(mapper)
                actor.GetProperty().SetColor(*color_rgb)
                actor.GetProperty().SetOpacity(opacity)
                actor.GetProperty().EdgeVisibilityOn()
                actor.GetProperty().SetEdgeColor(*color_rgb)
                actor.GetProperty().SetLineWidth(1.5)
                
                # Store identification properties
                actor.baseline_type = baseline_type
                actor.layer_path = baseline_data.get("layer_path", "")
                
                # Add actor to renderer
                if hasattr(self, 'renderer'):
                    self.renderer.AddActor(actor)
                    actors.append(actor)
        
        if actors:
            # Update the render window
            if hasattr(self, 'vtk_widget'):
                self.vtk_widget.GetRenderWindow().Render()
            
            self.message_text.append(f"✓ Generated {len(actors)} 3D plane segments for {baseline_type} (width: {width}m)")
        
        return actors

# ======================== Merger Marker methods =========================
    def add_vertical_marker_at_point(self, x, y, z, point_number, height=3):
        try:
            # Check if renderer exists
            if not hasattr(self, 'renderer') or not self.renderer:
                self.message_text.append("ERROR: Renderer not initialized")
                return False

            # Create a vertical line
            line_source = vtkLineSource()
            line_source.SetPoint1(x, y, z)  # Start at average point
            line_source.SetPoint2(x, y, z + height)  # Extend vertically upward
            
            # Create mapper
            mapper = vtkPolyDataMapper()
            mapper.SetInputConnection(line_source.GetOutputPort())
            
            # Create actor
            actor = vtkActor()
            actor.SetMapper(mapper)
            
            # Set color to ORANGE for all merger points (RGB: 1.0, 0.5, 0.0)
            actor.GetProperty().SetColor(1.0, 0.5, 0.0)  # Orange color
            actor.GetProperty().SetLineWidth(5)  # Thicker line for visibility
            
            # Add to renderer
            self.renderer.AddActor(actor)
            
            # Store the actor for later removal if needed
            if not hasattr(self, 'merger_markers'):
                self.merger_markers = []
            self.merger_markers.append(actor)
            
            # Add sphere at base
            sphere_source = vtkSphereSource()
            sphere_source.SetCenter(x, y, z)
            sphere_source.SetRadius(0.5)  # 0.5 meter radius
            
            sphere_mapper = vtkPolyDataMapper()
            sphere_mapper.SetInputConnection(sphere_source.GetOutputPort())
            
            sphere_actor = vtkActor()
            sphere_actor.SetMapper(sphere_mapper)
            sphere_actor.GetProperty().SetColor(1.0, 0.5, 0.0)  # Orange color
            
            self.renderer.AddActor(sphere_actor)
            self.merger_markers.append(sphere_actor)
            
            # OPTION 1: Use add_marker_label method (if you want to use it)
            self.add_marker_label(x, y, z + height + 2, f"MP{point_number}", (0.0, 0.0, 0.0))

            # Force render window update with modified camera for better view
            if hasattr(self, 'vtk_widget') and self.vtk_widget:
                # Reset camera to show all actors
                self.renderer.ResetCamera()
                # Adjust camera to get better view of markers
                camera = self.renderer.GetActiveCamera()
                camera.Zoom(0.8)  # Zoom out a bit
                self.vtk_widget.GetRenderWindow().Render()
        
            self.message_text.append(f"  Successfully added orange marker for Merger Point {point_number}")
            return True
        
        except Exception as e:
            self.message_text.append(f"ERROR adding vertical marker: {str(e)}")
            import traceback
            traceback.print_exc()
            return False
    
    def add_marker_label(self, x, y, z, text, color):
        try:
            from vtkmodules.vtkRenderingCore import vtkTextActor3D
            from vtkmodules.vtkCommonTransform import vtkTransform
            
            # Create vector text
            vector_text = vtkVectorText()
            vector_text.SetText(text)
            
            # Transform to position
            transform = vtkTransform()
            transform.Translate(x, y, z)
            transform.Scale(2, 2, 2)  # Scale text
            
            transform_filter = vtkTransformPolyDataFilter()
            transform_filter.SetTransform(transform)
            transform_filter.SetInputConnection(vector_text.GetOutputPort())
            
            # Create mapper and actor
            from vtkmodules.vtkRenderingCore import vtkPolyDataMapper, vtkActor
            mapper = vtkPolyDataMapper()
            mapper.SetInputConnection(transform_filter.GetOutputPort())
            
            actor = vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(color)
            
            # Add to renderer
            self.renderer.AddActor(actor)
            
            # Store for later removal
            if not hasattr(self, 'merger_labels'):
                self.merger_labels = []
            self.merger_labels.append(actor)
            
        except Exception as e:
            self.message_text.append(f"Error adding marker label: {str(e)}")

    def clear_merger_markers(self):
        """Clear all merger point markers from the 3D view."""
        if hasattr(self, 'merger_markers'):
            for actor in self.merger_markers:
                self.renderer.RemoveActor(actor)
            self.merger_markers.clear()
        
        if hasattr(self, 'merger_labels'):
            for actor in self.merger_labels:
                self.renderer.RemoveActor(actor)
            self.merger_labels.clear()

# ======================= Merger layer & heirarchy ends ================
#     def calculate_world_coordinates_from_chainage(self, chainage_m, relative_elevation_m):
#         """
#         Calculate world coordinates from chainage and relative elevation.
#         This should be implemented based on your coordinate system.
#         """
#         # This is a placeholder - implement based on your coordinate transformation
#         # You might need to use the zero line and other reference data
#         if hasattr(self, 'zero_start_point'):
#             # Simple linear interpolation along zero line
#             zero_dir = self.zero_end_point - self.zero_start_point
#             zero_length = np.linalg.norm(zero_dir)
#             if zero_length > 0:
#                 zero_dir_normalized = zero_dir / zero_length
#                 # Calculate position along zero line
#                 position = self.zero_start_point + zero_dir_normalized * chainage_m
#                 # Add elevation in Z direction
#                 position[2] += relative_elevation_m
#                 return position.tolist()
        
#         # Fallback: return chainage as X, 0 as Y, elevation as Z
#         return [chainage_m, 0.0, relative_elevation_m]

# =======================================================================================================================================
    def load_json_files_to_3d_pointcloud(self, folder_path):
        """
        Load only JSON files that are intended to be point clouds (e.g., not baseline or config files).
        """

        if not hasattr(self, 'vtk_widget') or not self.vtk_widget:
            self.message_text.append("Error: VTK widget not initialized.")
            return False

        renderer = self.vtk_widget.GetRenderWindow().GetRenderers().GetFirstRenderer()
        if not renderer:
            self.message_text.append("Error: No renderer found in VTK widget.")
            return False

        loaded_any = False

        # Define files to SKIP (baseline, config, zero line, etc.)
        skip_patterns = [
            "_baseline.json",
            "zero_line_config.json",
            "layer_config.txt",
            "Construction_Layer_config.txt",
            "worksheet_config.txt"
        ]

        for filename in os.listdir(folder_path):
            if not filename.lower().endswith('.json'):
                continue

            # Skip known non-point-cloud JSON files
            if any(pattern in filename.lower() for pattern in skip_patterns):
                continue  # Silently skip - no warning needed

            file_path = os.path.join(folder_path, filename)
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)

                points_list = data.get("points", [])
                if not points_list:
                    # Optional: log only if you want to debug unexpected files
                    # self.message_text.append(f"No 'points' key found in {filename} (skipped)")
                    continue

                # Convert list of [x,y,z] to numpy array
                points_array = np.array(points_list, dtype=np.float32)

                # Create VTK points
                vtk_points = vtk.vtkPoints()
                vtk_points.SetNumberOfPoints(len(points_array))

                for i, pt in enumerate(points_array):
                    vtk_points.SetPoint(i, pt[0], pt[1], pt[2])

                # Create polydata
                polydata = vtk.vtkPolyData()
                polydata.SetPoints(vtk_points)

                # Create vertex glyph for points
                vertex_filter = vtk.vtkVertexGlyphFilter()
                vertex_filter.SetInputData(polydata)

                # Mapper
                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputConnection(vertex_filter.GetOutputPort())

                # Actor
                actor = vtk.vtkActor()
                actor.SetMapper(mapper)
                actor.GetProperty().SetPointSize(3.0)
                actor.GetProperty().SetColor(0.8, 0.2, 0.2)  # red points

                actor.SetUserData({"source": f"PointCloud_{filename}"})

                renderer.AddActor(actor)
                loaded_any = True

                self.message_text.append(f"Loaded {len(points_array)} points from point cloud file: {filename}")

            except Exception as e:
                self.message_text.append(f"Failed to load {filename} as point cloud: {str(e)}")

        if loaded_any:
            self.vtk_widget.GetRenderWindow().Render()

        return loaded_any

# =======================================================================================================================================
    def clear_reference_lines(self):
        """Remove all reference lines from 2D graph"""
        if hasattr(self, 'reference_lines'):
            for line in self.reference_lines.values():
                line.remove()
            self.reference_lines.clear()
            self.canvas.draw_idle()

    def clear_reference_actors(self):
        """Remove all reference actors from 3D VTK view"""
        if not hasattr(self, 'vtk_widget') or not self.vtk_widget:
            return
        
        renderer = self.vtk_widget.GetRenderWindow().GetRenderers().GetFirstRenderer()
        if not renderer:
            return
        
        # Remove all actors from the stored list
        for actor in self.reference_actors:
            renderer.RemoveActor(actor)
        
        self.reference_actors.clear()
        
        # Also render the window
        self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================
# When creating a reference actor, add it to the list
    def create_reference_actor(self, poly_data):
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(poly_data)
        
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        
        # Mark it as a reference actor
        actor.is_reference_actor = True
        
        # Store in the list
        self.reference_actors.append(actor)
        
        # Add to renderer
        renderer = self.vtk_widget.GetRenderWindow().GetRenderers().GetFirstRenderer()
        renderer.AddActor(actor)
        
        return actor
    
# =======================================================================================================================================
    def load_zero_line_from_layer(self, layer_path):
        """Load zero_line_config.json from the given layer folder and fully update scale/graph"""
        json_path = os.path.join(layer_path, "zero_line_config.json")

        if not os.path.exists(json_path):
            self.message_text.append(f"Zero line config not found: {json_path}")
            return False

        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                config = json.load(f)

            # Debug: Show loaded keys
            self.message_text.append(f"Zero line config keys: {list(config.keys())}")

            # Extract values
            self.zero_start_point = np.array(config.get("point1", {}).get("coordinates", [0, 0, 0]))
            self.zero_end_point   = np.array(config.get("point2", {}).get("coordinates", [0, 0, 0]))
            self.zero_start_z     = config.get("reference_elevation_z", self.zero_start_point[2])
            self.total_distance   = config.get("total_length_m", 100.0)
            self.original_total_distance = self.total_distance
            self.zero_interval    = config.get("interval_m", 20.0)
            self.zero_start_km    = config.get("point1", {}).get("km")
            self.zero_end_km      = config.get("point2", {}).get("km")

            self.zero_line_set = config.get("zero_line_set", True)

            # Redraw zero line on graph
            if hasattr(self, 'zero_graph_line') and self.zero_graph_line:
                self.zero_graph_line.remove()
            self.zero_graph_line, = self.ax.plot(
                [0, self.total_distance], [0, 0],
                color='purple', linewidth=3, label='Zero Line'
            )

            # Update chainage ticks and scale
            self.update_chainage_ticks()

            # Show scale section
            if hasattr(self, 'scale_section'):
                self.scale_section.setVisible(True)

            # Update legend and redraw graph
            self.ax.legend()
            self.canvas.draw_idle()

            # Optional: Redraw zero line in 3D if implemented
            if hasattr(self, 'draw_zero_line_in_3d'):
                self.draw_zero_line_in_3d()

            self.message_text.append(f"Zero line loaded and graph updated from: {json_path}")
            self.message_text.append(f"  → Start KM: {self.zero_start_km}")
            self.message_text.append(f"  → Length: {self.total_distance:.2f} m")
            self.message_text.append(f"  → Interval: {self.zero_interval:.1f} m")

            return True

        except Exception as e:
            self.message_text.append(f"Error loading zero_line_config.json: {str(e)}")
            return False

# =======================================================================================================================================
    def load_all_baselines_from_layer(self, layer_path):
        """Load all *_baseline.json files from the given layer folder.
        Returns dict of loaded data.
        Does NOT draw on 2D graph — only stores data for 3D planes.
        """
        loaded = {}
        possible_files = [
            "surface_baseline.json",
            "construction_baseline.json",
            "road_surface_baseline.json",
            "deck_line_baseline.json",
            "projection_line_baseline.json",
            "material_baseline.json"
        ]

        key_map = {
            "surface_baseline.json": "surface",
            "construction_baseline.json": "construction",
            "road_surface_baseline.json": "road_surface",
            "deck_line_baseline.json": "deck_line",
            "projection_line_baseline.json": "projection_line",
            "material_baseline.json": "material",
        }

        for filename in possible_files:
            filepath = os.path.join(layer_path, filename)
            if not os.path.exists(filepath):
                continue

            try:
                with open(filepath, 'r', encoding='utf-8') as f:
                    data = json.load(f)

                ltype = key_map[filename]
                loaded[ltype] = data

                # Store polylines in memory (for 3D planes only)
                polylines_2d = []
                for poly in data.get("polylines", []):
                    poly_2d = [(pt["chainage_m"], pt["relative_elevation_m"]) for pt in poly["points"]]
                    if len(poly_2d) >= 2:
                        polylines_2d.append(poly_2d)

                self.line_types[ltype]['polylines'] = polylines_2d

                # IMPORTANT: Do NOT redraw on graph here!

            except Exception as e:
                self.message_text.append(f"Error loading {filename}: {str(e)}")

        return loaded

# =======================================================================================================================================
    def redraw_baseline_on_graph(self, ltype, style="solid"):
        """Redraw a loaded baseline on the 2D matplotlib graph.
        Supports 'solid' or 'dotted' style.
        """
        color = self.line_types[ltype]['color']
        linestyle = '--' if style == "dotted" else '-'

        for poly_2d in self.line_types[ltype]['polylines']:
            if len(poly_2d) < 2:
                continue
            xs = [p[0] for p in poly_2d]
            ys = [p[1] for p in poly_2d]
            line, = self.ax.plot(
                xs, ys,
                color=color,
                linewidth=2.5,
                linestyle=linestyle,
                label=f"{ltype.capitalize()} ({style})"
            )
            self.line_types[ltype]['artists'].append(line)

        self.ax.legend()
        self.canvas.draw_idle()

# =======================================================================================================================================
    def map_baselines_to_3d_planes_from_data(self, loaded_baselines, width):
        """Regenerate 3D planes from loaded baseline data (same logic as map_baselines_to_3d_planes but from dict)"""
        if not self.zero_line_set:
            return

        half_width = width / 2.0
        zero_dir_vec = self.zero_end_point - self.zero_start_point
        zero_length = self.total_distance
        ref_z = self.zero_start_z

        for ltype, baseline_data in loaded_baselines.items():
            rgba = self.plane_colors.get(ltype, (0.5, 0.5, 0.5, 0.4))
            color_rgb = rgba[:3]
            opacity = rgba[3]

            for poly in baseline_data.get("polylines", []):
                points_3d = poly["points"]
                if len(points_3d) < 2:
                    continue

                for i in range(len(points_3d) - 1):
                    pt1 = points_3d[i]
                    pt2 = points_3d[i + 1]

                    center1 = np.array(pt1["world_coordinates"])
                    center2 = np.array(pt2["world_coordinates"])

                    seg_dir = center2 - center1
                    seg_len = np.linalg.norm(seg_dir)
                    if seg_len < 1e-6:
                        continue
                    seg_unit = seg_dir / seg_len

                    horiz = np.array([seg_unit[0], seg_unit[1], 0.0])
                    hlen = np.linalg.norm(horiz)
                    if hlen < 1e-6:
                        zero_unit = zero_dir_vec / np.linalg.norm(zero_dir_vec)
                        perp = np.array([-zero_unit[1], zero_unit[0], 0.0])
                    else:
                        horiz /= hlen
                        perp = np.array([-horiz[1], horiz[0], 0.0])

                    perp_len = np.linalg.norm(perp)
                    if perp_len > 0:
                        perp /= perp_len

                    c1 = center1 + perp * half_width
                    c2 = center1 - perp * half_width
                    c3 = center2 - perp * half_width
                    c4 = center2 + perp * half_width

                    plane = vtkPlaneSource()
                    plane.SetOrigin(c1[0], c1[1], c1[2])
                    plane.SetPoint1(c4[0], c4[1], c4[2])
                    plane.SetPoint2(c2[0], c2[1], c2[2])
                    plane.SetXResolution(12)
                    plane.SetYResolution(2)
                    plane.Update()

                    mapper = vtkPolyDataMapper()
                    mapper.SetInputConnection(plane.GetOutputPort())

                    actor = vtkActor()
                    actor.SetMapper(mapper)
                    actor.GetProperty().SetColor(*color_rgb)
                    actor.GetProperty().SetOpacity(opacity)
                    actor.GetProperty().EdgeVisibilityOn()
                    actor.GetProperty().SetEdgeColor(*color_rgb)
                    actor.GetProperty().SetLineWidth(1.5)

                    self.renderer.AddActor(actor)
                    self.baseline_plane_actors.append(actor)

        self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================
    def show_graph_section(self, category):
        """
        Ensures the 2D cross-section graph and baseline controls are fully visible
        when opening an existing worksheet.
        """
        # 1. Show the entire bottom section (contains line section + graph)
        if hasattr(self, 'bottom_section'):
            self.bottom_section.setVisible(True)

        # 2. Show the collapsible line section (checkboxes)
        if hasattr(self, 'line_section'):
            self.line_section.setVisible(True)

        # 3. Expand the line section if it's collapsed
        if hasattr(self, 'line_content_widget') and not self.line_content_widget.isVisible():
            self.line_content_widget.show()
            self.collapse_button.setText("◀")
            self.collapse_button.setToolTip("Close Line Section")
            self.undo_button.show()
            self.redo_button.show()
            self.line_section.setMaximumWidth(350)
            self.line_section.setFixedWidth(350)

        # 4. Show scale section (chainage slider + scale bar)
        if hasattr(self, 'scale_section'):
            self.scale_section.setVisible(True)

        # 5. Show relevant baseline containers based on category
        if category == "Road":
            self.surface_container.setVisible(True)
            self.construction_container.setVisible(True)
            self.road_surface_container.setVisible(True)
            self.zero_container.setVisible(True)
        elif category == "Bridge":
            self.deck_line_container.setVisible(True)
            self.projection_container.setVisible(True)
            self.construction_dots_container.setVisible(True)
            if hasattr(self, 'bridge_zero_container'):
                self.bridge_zero_container.setVisible(True)

        # 6. Show action buttons
        self.preview_button.setVisible(True)
        self.elivation_angle_button.setVisible(True)
        self.threed_map_button.setVisible(True)
        self.save_button.setVisible(True)

        # 7. Force redraw of matplotlib canvas
        if hasattr(self, 'canvas'):
            self.canvas.draw_idle()

        # 8. Force layout update
        QApplication.processEvents()

# =======================================================================================================================================
    def load_point_cloud_files(self, file_list):
        """Load multiple point cloud files (merge or first one) - currently loads first file with progress bar"""
        if not file_list:
            return

        # For simplicity, load first file
        first_file = file_list[0]
        file_path = first_file  # For consistency with the single-load method

        # Store the loaded file path and name (same as single load)
        self.loaded_file_path = file_path
        self.loaded_file_name = os.path.splitext(os.path.basename(file_path))[0]

        try:
            # Show progress bar with file info
            self.show_progress_bar(file_path)
            self.update_progress(10, "Starting file loading...")

            self.update_progress(30, "Loading point cloud data...")
            self.point_cloud = o3d.io.read_point_cloud(file_path)

            if self.point_cloud.is_empty():
                raise ValueError("Point cloud is empty!")

            self.update_progress(50, "Converting to VTK format...")

            # Display in VTK
            points = np.asarray(self.point_cloud.points)
            colors = np.asarray(self.point_cloud.colors) if self.point_cloud.has_colors() else None

            poly_data = vtk.vtkPolyData()
            vtk_points = vtk.vtkPoints()
            vertices = vtk.vtkCellArray()

            for i, pt in enumerate(points):
                vtk_points.InsertNextPoint(pt)
                vertex = vtk.vtkVertex()
                vertex.GetPointIds().SetId(0, i)
                vertices.InsertNextCell(vertex)

            poly_data.SetPoints(vtk_points)
            poly_data.SetVerts(vertices)

            if colors is not None:
                self.update_progress(70, "Processing colors...")
                vtk_colors = vtk.vtkUnsignedCharArray()
                vtk_colors.SetNumberOfComponents(3)
                vtk_colors.SetName("Colors")
                for c in (colors * 255).astype(np.uint8):
                    vtk_colors.InsertNextTuple(c)
                poly_data.GetPointData().SetScalars(vtk_colors)
            else:
                self.update_progress(70, "Preparing visualization...")

            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(poly_data)

            self.update_progress(90, "Creating visualization...")

            if self.point_cloud_actor:
                self.renderer.RemoveActor(self.point_cloud_actor)

            self.point_cloud_actor = vtk.vtkActor()
            self.point_cloud_actor.SetMapper(mapper)
            self.point_cloud_actor.GetProperty().SetPointSize(2)

            self.renderer.AddActor(self.point_cloud_actor)
            self.renderer.ResetCamera()
            self.vtk_widget.GetRenderWindow().Render()

            self.update_progress(100, "Loading complete!")
            QTimer.singleShot(500, self.hide_progress_bar)

            self.message_text.append(f"Successfully loaded point cloud: {os.path.basename(file_path)}")

        except Exception as e:
            self.hide_progress_bar()
            self.message_text.append(f"Failed to load point cloud '{os.path.basename(file_path)}': {str(e)}")
            QMessageBox.warning(self, "Load Failed", f"Could not load point cloud:\n{file_path}\n\nError: {str(e)}")

# =======================================================================================================================================
    def show_help_dialog(self):
        """Open the Help Dialog when Help button is clicked"""
        dialog = HelpDialog(self)
        dialog.exec_()  # Modal – blocks until closed

    def handle_design_button_click(self):
    #     if not self.merger_frame.isVisible():
    #         self.merger_frame.setVisible(True)

        if self.three_D_frame.isVisible():
            self.three_D_frame.setVisible(False)

        # Hide measurement layers when switching to design
        if self.checkboxes.isVisible():
            self.checkboxes.setVisible(False)

        self.main_measurement_section.setVisible(False)

        if self.herarchy_section.isVisible():
            self.herarchy_section.setVisible(False)

    def handle_construction_button_click(self):
    #     if not self.merger_frame.isVisible():
    #         self.merger_frame.setVisible(True)

        if self.three_D_frame.isVisible():
            self.three_D_frame.setVisible(False)

        # Hide measurement layers when switching to design
        if self.checkboxes.isVisible():
            self.checkboxes.setVisible(False)

        self.main_measurement_section.setVisible(False)

        if self.herarchy_section.isVisible():
            self.herarchy_section.setVisible(False)

    def handle_measurement_button_click(self):
        # if self.merger_frame.isVisible():
        #     self.merger_frame.setVisible(False)
        self.right_section.setMinimumHeight(900)

        if self.two_D_frame.isVisible():
            self.two_D_frame.setVisible(False)

        if self.checkboxes.isVisible():
            self.checkboxes.setVisible(True)

        if self.scale_section.isVisible():
            self.scale_section.setVisible(False)

        if self.bottom_section.isVisible():
            self.bottom_section.setVisible(False)

        if self.herarchy_section.isVisible():
            self.herarchy_section.setVisible(False)

# =======================================================================================================================================
    def save_current_lines_state(self):
        """Save the current graph lines state when switching between modes"""
        if self.current_mode == 'road':
            # Save road lines
            self.road_lines_data = {
                'construction': {
                    'polylines': self.line_types['construction']['polylines'].copy(),
                    'artists': []  # Don't save artist references directly
                },
                'surface': {
                    'polylines': self.line_types['surface']['polylines'].copy(),
                    'artists': []
                },
                'road_surface': {
                    'polylines': self.line_types['road_surface']['polylines'].copy(),
                    'artists': []
                },
                'zero': {
                    'polylines': self.line_types['zero']['polylines'].copy(),
                    'artists': []
                }
            }
            self.message_text.append("Road lines state saved")
            
        elif self.current_mode == 'bridge':
            # Save bridge lines
            self.bridge_lines_data = {
                'deck_line': {
                    'polylines': self.line_types['deck_line']['polylines'].copy(),
                    'artists': []
                },
                'projection_line': {
                    'polylines': self.line_types['projection_line']['polylines'].copy(),
                    'artists': []
                },
                'construction_dots': {
                    'polylines': self.line_types['construction_dots']['polylines'].copy(),
                    'artists': []
                },
                'zero': {
                    'polylines': self.line_types['zero']['polylines'].copy(),
                    'artists': []
                }
            }
            self.message_text.append("Bridge lines state saved")

# =======================================================================================================================================
    def restore_saved_lines_state(self, mode):
        """Restore saved graph lines for a specific mode"""
        if mode == 'road' and self.road_lines_data:
            # Clear current lines first
            self.clear_graph_for_mode(mode)
            
            # Restore road lines
            for line_type in ['construction', 'surface', 'road_surface', 'zero']:
                if line_type in self.road_lines_data:
                    self.line_types[line_type]['polylines'] = self.road_lines_data[line_type]['polylines'].copy()
                    
                    # Recreate artists for the polylines
                    for polyline in self.line_types[line_type]['polylines']:
                        if len(polyline) > 1:
                            xs = [p[0] for p in polyline]
                            ys = [p[1] for p in polyline]
                            color = self.line_types[line_type]['color']
                            artist, = self.ax.plot(xs, ys, color=color, linewidth=2, marker='o', markersize=5)
                            self.line_types[line_type]['artists'].append(artist)
                            
                            # Add to all_graph_lines for undo/redo
                            self.all_graph_lines.append((line_type, polyline.copy(), artist, None))
            
            self.message_text.append("Restored saved road lines")
            
        elif mode == 'bridge' and self.bridge_lines_data:
            # Clear current lines first
            self.clear_graph_for_mode(mode)
            
            # Restore bridge lines
            for line_type in ['deck_line', 'projection_line', 'construction_dots', 'zero']:
                if line_type in self.bridge_lines_data:
                    self.line_types[line_type]['polylines'] = self.bridge_lines_data[line_type]['polylines'].copy()
                    
                    # Recreate artists for the polylines
                    for polyline in self.line_types[line_type]['polylines']:
                        if len(polyline) > 1:
                            xs = [p[0] for p in polyline]
                            ys = [p[1] for p in polyline]
                            color = self.line_types[line_type]['color']
                            artist, = self.ax.plot(xs, ys, color=color, linewidth=2, marker='o', markersize=5)
                            self.line_types[line_type]['artists'].append(artist)
                            
                            # Special handling for construction dots labels
                            if line_type == 'construction_dots':
                                for i, (x, y) in enumerate(polyline, 1):
                                    label = self.add_point_label(x, y, i, line_type)
                                    if label:
                                        self.point_labels.append(label)
                            
                            # Add to all_graph_lines for undo/redo
                            self.all_graph_lines.append((line_type, polyline.copy(), artist, None))
            
            self.message_text.append("Restored saved bridge lines")
        
        # Redraw canvas
        self.canvas.draw()
        self.figure.tight_layout()

# =======================================================================================================================================
    def clear_graph_for_switch(self):
        """Clear graph when switching between road and bridge modes"""
        # Clear current drawing session
        self.current_points = []
        if self.current_artist is not None:
            try:
                self.current_artist.remove()
            except:
                pass
            self.current_artist = None
        self.current_redo_points = []
        
        # Clear current point labels
        for label in self.current_point_labels:
            try:
                if label in self.ax.texts:
                    label.remove()
            except:
                pass
        self.current_point_labels = []
        
        # Disconnect drawing events if connected
        if self.cid_click is not None:
            self.canvas.mpl_disconnect(self.cid_click)
            self.cid_click = None
        if self.cid_key is not None:
            self.canvas.mpl_disconnect(self.cid_key)
            self.cid_key = None
        
        # Uncheck all checkboxes
        self.construction_line.setChecked(False)
        self.surface_baseline.setChecked(False)
        if hasattr(self, 'road_surface_line'):
            self.road_surface_line.setChecked(False)
        if hasattr(self, 'construction_dots_line'):
            self.construction_dots_line.setChecked(False)
        if hasattr(self, 'deck_line'):
            self.deck_line.setChecked(False)
        if hasattr(self, 'projection_line'):
            self.projection_line.setChecked(False)
        
        # Reset active line type
        self.active_line_type = None
        
        self.message_text.append("Graph cleared for mode switch")

# =======================================================================================================================================
    def clear_graph_for_mode(self, mode):
        """Clear graph lines for specific mode only"""
        if mode == 'road':
            # Clear road-specific lines
            for line_type in ['construction', 'surface', 'road_surface']:
                self.clear_line_type(line_type)
        elif mode == 'bridge':
            # Clear bridge-specific lines
            for line_type in ['deck_line', 'projection_line', 'construction_dots']:
                self.clear_line_type(line_type)

# =======================================================================================================================================
    def clear_line_type(self, line_type):
        """Clear a specific line type from the graph"""
        if line_type in self.line_types:
            # Remove artists
            for artist in self.line_types[line_type]['artists']:
                try:
                    if artist in self.ax.lines or artist in self.ax.collections:
                        artist.remove()
                except:
                    pass
            
            # Clear the lists
            self.line_types[line_type]['artists'] = []
            self.line_types[line_type]['polylines'] = []
            
            # Remove from all_graph_lines
            new_all_graph_lines = []
            for item in self.all_graph_lines:
                lt, points, artist, ann = item
                if lt != line_type:
                    new_all_graph_lines.append(item)
                else:
                    # Remove the annotation if it exists
                    if ann:
                        try:
                            ann.remove()
                        except:
                            pass
            
            self.all_graph_lines = new_all_graph_lines
            
            # For construction dots, also remove labels
            if line_type == 'construction_dots':
                texts_to_remove = []
                for text in self.ax.texts:
                    if hasattr(text, 'point_data') and text.point_data.get('line_type') == 'construction_dots':
                        texts_to_remove.append(text)
                
                for text in texts_to_remove:
                    try:
                        text.remove()
                    except:
                        pass
        
# =======================================================================================================================================
# ON CHECKBOX CHANGED 
    def on_checkbox_changed(self, state, line_type):
        if state == Qt.Checked:
            if line_type == 'construction_dots':
                
                # Don't finish previous line automatically - let user double-click to complete
                self.active_line_type = line_type
                
                # Set up event listeners if not already set up
                if self.cid_click is None:
                    self.cid_click = self.canvas.mpl_connect('button_press_event', self.on_draw_click)
                    self.cid_key = self.canvas.mpl_connect('key_press_event', self.on_key_press)
                
                self.current_points = []
                self.current_artist = None
                self.current_redo_points = []
                
                self.message_text.append("Construction Dots Mode: Click on graph to add construction points")
                self.message_text.append("Double-click to complete the line")
                self.message_text.append("Click on the P1, P2 labels at the top to configure each point")
                
                self.canvas.draw()
                self.figure.tight_layout()
                return
            
            if line_type == 'zero':
                # Zero line handling remains the same
                if self.zero_line_set:
                    if self.zero_start_actor:
                        self.zero_start_actor.SetVisibility(True)
                    if self.zero_end_actor:
                        self.zero_end_actor.SetVisibility(True)
                    if self.zero_line_actor:
                        self.zero_line_actor.SetVisibility(True)
                    self.total_distance = self.zero_physical_dist
                    self.ax.set_xlim(0, self.total_distance)
                    self.update_chainage_ticks()

                    # Show scale section if zero line is set
                    if self.scale_section:
                        self.scale_section.setVisible(True)
                            
                    if self.zero_graph_line:
                        self.zero_graph_line.set_visible(True)
                    self.canvas.draw()
                    self.figure.tight_layout()
                    self.volume_slider.setValue(0)
                    self.update_scale_marker()
                    return
                else:
                    self.set_measurement_type('zero_line')
                    self.zero_points = []
                    self.temp_zero_actors = []
                    self.message_text.append("Click two points on the point cloud to set zero line start and end.")
                    return           

            # First, if there's an ongoing line of different type, finish it
            if self.active_line_type and self.active_line_type != line_type and self.current_points:
                self.message_text.append(f"Finishing {self.active_line_type.replace('_', ' ').title()} before switching to {line_type.replace('_', ' ').title()}")
                self.finish_current_polyline()
            
            self.active_line_type = line_type
            if self.cid_click is None:
                self.cid_click = self.canvas.mpl_connect('button_press_event', self.on_draw_click)
                self.cid_key = self.canvas.mpl_connect('key_press_event', self.on_key_press)
            
            self.current_points = []
            self.current_artist = None
            self.current_redo_points = []
            
            # Add instruction message
            line_names = {
                'surface': 'Surface Line',
                'construction': 'Construction Line',
                'road_surface': 'Road Surface Line',
                'deck_line': 'Deck Line',
                'projection_line': 'Projection Line'
            }
            if line_type in line_names:
                self.message_text.append(f"{line_names[line_type]} Mode: Click to add points, double-click to complete")
            
            self.canvas.draw()
            self.figure.tight_layout()
        
        else:  # State == Qt.Unchecked
            if line_type == 'zero':
                if self.zero_line_set:
                    if self.zero_start_actor:
                        self.zero_start_actor.SetVisibility(False)
                    if self.zero_end_actor:
                        self.zero_end_actor.SetVisibility(False)
                    if self.zero_line_actor:
                        self.zero_line_actor.SetVisibility(False)
                    if self.zero_graph_line:
                        self.zero_graph_line.set_visible(False)
                            
                    # Hide scale section when zero line is unchecked
                    if hasattr(self, 'scale_section'):
                        self.scale_section.setVisible(False)
                    
                    # Reset scale section to default format
                    if hasattr(self, 'scale_ax') and self.scale_ax is not None:
                        self.scale_ax.set_xticks(np.arange(0, self.total_distance + 1, 5))
                        self.scale_ax.set_xticklabels([f"{x:.0f}" for x in np.arange(0, self.total_distance + 1, 5)])
                        self.scale_canvas.draw()
                return
            
            # For other line types when unchecked
            if self.active_line_type == line_type and self.current_points:
                self.message_text.append(f"Finishing {line_type.replace('_', ' ').title()} before unchecking")
                self.finish_current_polyline()
            
            # Only disconnect if no non-zero checkboxes are checked
            if not (self.construction_line.isChecked() or self.surface_baseline.isChecked() or 
                    self.road_surface_line.isChecked() or self.deck_line.isChecked() or 
                    self.projection_line.isChecked() or self.construction_dots_line.isChecked()):
                
                if self.cid_click is not None:
                    self.canvas.mpl_disconnect(self.cid_click)
                    self.cid_click = None
                if self.cid_key is not None:
                    self.canvas.mpl_disconnect(self.cid_key)
                    self.cid_key = None
                
                self.active_line_type = None
                self.current_points = []
                if self.current_artist is not None:
                    self.current_artist.remove()
                    self.current_artist = None
                
                self.canvas.draw()
                self.figure.tight_layout()

# =======================================================================================================================================
# UPDATE CHAINAGE TICKS ON GRAPHS
    def update_chainage_ticks(self):
        """Update both the top Chainage Scale and the main graph X-axis with KM+interval ticks."""
        if not (self.zero_line_set and hasattr(self, 'zero_interval') and self.zero_interval and
                hasattr(self, 'zero_start_km') and self.zero_start_km is not None):
            # Fallback to simple distance if no proper chainage config
            if hasattr(self, 'scale_ax') and self.scale_ax:
                self.scale_ax.clear()
                self.scale_ax.axis('off')
                self.scale_canvas.draw_idle() if hasattr(self, 'scale_canvas') else None
            self.ax.set_xlabel('Distance (m)')
            self.ax.set_xticks(np.linspace(0, self.total_distance, 10))
            self.ax.set_xticklabels([f"{int(x)}m" for x in np.linspace(0, self.total_distance + 1, 10)])
            self.canvas.draw_idle()
            return

        # Calculate tick positions
        num_intervals = int(self.total_distance / self.zero_interval)
        tick_positions = [i * self.zero_interval for i in range(num_intervals + 1)]
        tick_labels = []
        current_km = self.zero_start_km
        remaining = self.total_distance
        for i in range(num_intervals + 1):
            interval_value = i * self.zero_interval
            if interval_value >= 1000:
                additional_km = int(interval_value // 1000)
                current_km += additional_km
                interval_value %= 1000
            label = f"{current_km}+{int(interval_value):03d}"
            tick_labels.append(label)

        # === UPDATE TOP CHAINAGE SCALE (Orange bar) ===
        if hasattr(self, 'scale_ax') and self.scale_ax:
            self.scale_ax.clear()
            self.scale_ax.plot([0, self.total_distance], [0, 0], color='black', linewidth=3)
            self.scale_ax.set_xlim(0, self.total_distance)
            self.scale_ax.set_ylim(-0.1, 1.2)
            self.scale_ax.set_yticks([])
            self.scale_ax.set_xticks(tick_positions)
            self.scale_ax.set_xticklabels(tick_labels, rotation=15, ha='right', fontsize=8)
            self.scale_ax.set_title("Chainage Scale", fontsize=10, pad=10, color='#D35400')
            self.scale_ax.spines['bottom'].set_visible(True)
            self.scale_ax.spines['top'].set_visible(False)
            self.scale_ax.spines['left'].set_visible(False)
            self.scale_ax.spines['right'].set_visible(False)
            self.scale_ax.tick_params(axis='x', which='both', length=8, width=1, colors='black')
            self.scale_ax.set_facecolor('#FFE5B4')  # Light orange background for better visibility
            if hasattr(self, 'scale_canvas'):
                self.scale_canvas.draw_idle()

        # === UPDATE MAIN GRAPH X-AXIS ===
        self.ax.set_xlim(0, self.total_distance)
        self.ax.set_xticks(tick_positions)
        self.ax.set_xticklabels(tick_labels, rotation=15, ha='right', fontsize=8)
        self.ax.set_xlabel('Chainage (KM + Interval)', fontsize=10, labelpad=10)
        self.ax.grid(True, axis='x', linestyle='--', alpha=0.6, linewidth=1.2)

        # Improve Y-axis readability
        self.ax.tick_params(axis='y', labelsize=10)
        self.ax.set_ylabel('Relative Elevation (m)', fontsize=10, labelpad=10)

        self.canvas.draw_idle()

# =======================================================================================================================================
# UPDATE SCALE MARKER BASED ON SLIDER
    def update_scale_marker(self):
        """Update red marker and chainage label when volume slider moves.
        Now snaps to nearest sub-interval (half of main interval) for better alignment."""
        if not self.zero_line_set or not hasattr(self, 'scale_ax'):
            return

        value = self.volume_slider.value()
        pos = value / 100.0 * self.total_distance

        interval = self.zero_interval
        half_interval = interval / 2.0

        # Snap to nearest half-interval
        snapped_pos = round(pos / half_interval) * half_interval
        snapped_pos = max(0, min(snapped_pos, self.total_distance))  # Clamp

        # Update vertical red line
        self.scale_marker.set_data([snapped_pos, snapped_pos], [0, 1])

        # === SAFE CHAINAGE LABEL ===
        start_km = getattr(self, 'zero_start_km', 0)

        meters_into_line = snapped_pos
        km_offset = int(meters_into_line // 1000)
        chainage_m = int(meters_into_line % 1000)

        marker_label = f"Chainage: {start_km + km_offset}+{chainage_m:03d}"

        # Remove old label
        if hasattr(self, 'scale_marker_label'):
            try:
                self.scale_marker_label.remove()
            except:
                pass

        # Add new label
        self.scale_marker_label = self.scale_ax.text(
            snapped_pos, 1.1, marker_label,
            color='red', fontsize=10, fontweight='bold',
            ha='center', va='bottom',
            bbox=dict(boxstyle="round,pad=0.3", facecolor="white",
                      edgecolor="red", alpha=0.9, linewidth=1)
        )

        # Update bottom indicator line
        if not hasattr(self, 'scale_marker_bottom'):
            self.scale_marker_bottom, = self.scale_ax.plot([snapped_pos, snapped_pos], [0, 0.1], color='red', linewidth=2)
        else:
            self.scale_marker_bottom.set_data([snapped_pos, snapped_pos], [0, 0.1])

        self.scale_canvas.draw()

# =======================================================================================================================================
    def update_main_graph_marker(self, slider_value):
        """Update orange vertical marker on main 2D graph"""
        if not self.zero_line_set:
            return

        pos = slider_value / 100.0 * self.total_distance

        # Update vertical line
        if not hasattr(self, 'main_graph_marker'):
            self.main_graph_marker = self.ax.axvline(x=pos, color='orange',
                                                     linestyle='--', alpha=0.7, linewidth=2)
        else:
            self.main_graph_marker.set_xdata([pos, pos])

        # Get safe chainage label
        chainage_label = self.get_chainage_label(pos)

        # Update label
        if not hasattr(self, 'main_graph_marker_label'):
            self.main_graph_marker_label = self.ax.text(
                pos, self.ax.get_ylim()[1] * 0.95,
                f"← Chainage: {chainage_label}",
                color='orange', fontweight='bold', ha='right',
                bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8)
            )
        else:
            self.main_graph_marker_label.set_position((pos, self.ax.get_ylim()[1] * 0.95))
            self.main_graph_marker_label.set_text(f"← Chainage: {chainage_label}")

        self.canvas.draw_idle()

# =======================================================================================================================================
    def on_graph_scrolled(self):
        """Update slider position when graph is manually scrolled"""
        if not hasattr(self, 'graph_horizontal_scrollbar') or not self.volume_slider:
            return
        
        scrollbar = self.graph_horizontal_scrollbar
        slider = self.volume_slider
        
        # Only update if slider is not being dragged
        if not slider.isSliderDown():
            scroll_value = scrollbar.value()
            scroll_max = scrollbar.maximum()
            
            if scroll_max > 0:
                slider_value = int((scroll_value / scroll_max) * slider.maximum())
                slider.blockSignals(True)  # Temporarily block signals to prevent loop
                slider.setValue(slider_value)
                slider.blockSignals(False)
                
                # Update the visual elements
                if self.zero_line_set:
                    self.update_scale_marker()
                    self.update_main_graph_marker(slider_value)

# =======================================================================================================================================
    def complete_line_with_double_click(self):
        """Complete the current line when double-clicked"""
        if self.active_line_type and len(self.current_points) > 1:
            self.finish_current_polyline()
            self.message_text.append(f"{self.active_line_type.replace('_', ' ').title()} completed with double-click")
        elif self.active_line_type and len(self.current_points) == 1:
            self.message_text.append("Need at least 2 points to create a line. Add more points before double-clicking.")              

# =======================================================================================================================================
# Add a helper method to format chainage labels:
    def get_chainage_label(self, position):
        """Return formatted chainage string like '101+020' for any position"""
        if not self.zero_line_set or not hasattr(self, 'zero_interval') or self.zero_interval <= 0:
            return f"{position:.1f}m"

        interval = int(self.zero_interval)
        start_km = getattr(self, 'zero_start_km', 0) if hasattr(self, 'zero_start_km') else 0

        interval_number = int(round(position / interval))
        interval_value_meters = interval_number * interval

        return f"{start_km}+{interval_value_meters:03d}"

# =======================================================================================================================================
# UPDATE ZERO ACTORS
    def update_zero_actors(self):
        if not self.zero_line_set:
            return
        # Remove old actors
        if self.zero_start_actor:
            self.renderer.RemoveActor(self.zero_start_actor)
            if self.zero_start_actor in self.measurement_actors:
                self.measurement_actors.remove(self.zero_start_actor)
        if self.zero_end_actor:
            self.renderer.RemoveActor(self.zero_end_actor)
            if self.zero_end_actor in self.measurement_actors:
                self.measurement_actors.remove(self.zero_end_actor)
        if self.zero_line_actor:
            self.renderer.RemoveActor(self.zero_line_actor)
            if self.zero_line_actor in self.measurement_actors:
                self.measurement_actors.remove(self.zero_line_actor)
        
        # Recreate actors
        self.zero_start_actor = self.add_sphere_marker(self.zero_start_point, "Start", color="purple")
        self.zero_end_actor = self.add_sphere_marker(self.zero_end_point, "End", color="purple")
        self.zero_line_actor = self.add_line_between_points(self.zero_start_point, self.zero_end_point, "purple", show_label=False)
        self.zero_physical_dist = np.linalg.norm(self.zero_end_point - self.zero_start_point)
        self.total_distance = self.zero_physical_dist
        self.zero_start_z = self.zero_start_point[2] # Set reference zero elevation to Point_1 Z
        
        # Update both main graph and scale graph
        self.ax.set_xlim(0, self.total_distance)
        self.scale_ax.set_xlim(0, self.total_distance)
        
        if self.zero_graph_line:
            self.zero_graph_line.remove()
        self.zero_graph_line, = self.ax.plot([0, self.total_distance], [0, 0], color='purple', linewidth=3)
        
        # Update scale line
        self.scale_line.set_data([0, self.total_distance], [0.5, 0.5])
        
        # Update ticks for both graphs
        self.update_chainage_ticks()
        self.update_scale_ticks()  # Add this line
        
        self.volume_slider.setValue(0)
        self.scale_marker.set_data([0, 0], [0, 1])
        self.scale_canvas.draw()
        self.canvas.draw()
        self.figure.tight_layout()
        self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================
# In the update_scale_ticks method, improve the tick labels:
    def update_scale_ticks(self):
        """Rebuild the chainage scale below the 3D view with correct KM+Interval labels"""
        if not self.zero_line_set or not hasattr(self, 'zero_interval') or self.zero_interval <= 0:
            return

        self.scale_ax.clear()

        interval = int(self.zero_interval)
        start_km = getattr(self, 'zero_start_km', 0) if hasattr(self, 'zero_start_km') else 0

        tick_positions = []
        tick_labels = []

        current_pos = 0
        while current_pos <= self.total_distance + interval:
            tick_positions.append(current_pos)

            interval_number = int(round(current_pos / interval))
            interval_value_meters = interval_number * interval

            tick_labels.append(f"{start_km}+{interval_value_meters:03d}")

            current_pos += interval

        # Rebuild scale line and marker
        self.scale_line, = self.scale_ax.plot([0, self.total_distance], [0.5, 0.5],
                                              color='black', linewidth=3)
        self.scale_marker, = self.scale_ax.plot([0, 0], [0, 1], color='red',
                                                linewidth=2, linestyle='--')

        self.scale_ax.set_xticks(tick_positions)
        self.scale_ax.set_xticklabels(tick_labels, rotation=30, ha='right')
        self.scale_ax.set_xlim(0, self.total_distance)
        self.scale_ax.set_ylim(0, 1.2)
        self.scale_ax.set_yticks([])
        self.scale_ax.set_xlabel('Chainage (KM+Interval)', labelpad=10, fontweight='bold')
        self.scale_ax.set_title('Chainage Scale', fontweight='bold', pad=3)
        self.scale_ax.grid(True, axis='x', linestyle='--', alpha=0.3)
        self.scale_ax.spines['top'].set_visible(False)
        self.scale_ax.spines['right'].set_visible(False)
        self.scale_ax.spines['left'].set_visible(False)

        self.scale_section.setVisible(True)
        self.scale_canvas.draw()

# =======================================================================================================================================
# EDIT ZERO LINE
    def edit_zero_line(self):
        if not self.zero_line_set:
            return
        dialog = ZeroLineDialog(
            self.zero_start_point, self.zero_end_point,
            self.zero_start_km, self.zero_start_chain,
            self.zero_end_km, self.zero_end_chain,
            self.zero_interval,
            self
        )
        if dialog.exec_() == QDialog.Accepted:
            try:
                p1, p2 = dialog.get_points()
                if p1 is not None and p2 is not None:
                    self.zero_start_point = p1
                    self.zero_end_point = p2
                
                # Store the configuration
                self.zero_start_km = int(dialog.km1_edit.text() or 0)
                self.zero_start_chain = float(dialog.chain1_edit.text() or 0)
                self.zero_end_km = int(dialog.km2_edit.text() or 0)
                self.zero_end_chain = float(dialog.chain2_edit.text() or 0)
                self.zero_interval = int(dialog.interval_edit.text() or 20)
                
                # IMPORTANT: Calculate the total chainage distance
                # Convert both to absolute meters for calculation
                start_abs_m = self.zero_start_km * 1000 + self.zero_start_chain
                end_abs_m = self.zero_end_km * 1000 + self.zero_end_chain
                self.zero_total_chainage_m = end_abs_m - start_abs_m
                
                # Update visual elements
                self.update_zero_actors()
                self.update_chainage_ticks()
                
                # Update scale section with proper formatting
                self.update_scale_ticks()
                
                # Update marker position
                current_slider_value = self.volume_slider.value()
                self.update_scale_marker()
                self.update_main_graph_marker(current_slider_value)
                
                # Make sure scale section is visible
                self.scale_section.setVisible(True)
                
                self.canvas.draw()
                self.figure.tight_layout()
                self.message_text.append("Zero line configuration updated.")
                
            except ValueError:
                QMessageBox.warning(self, "Invalid Input", "Please enter valid numbers.")

# =======================================================================================================================================
    def edit_construction_dots_line(self):
        if not self.construction_dots_line.isChecked():
            return
        
        # Check if bridge baseline is active
        if self.bridge_baseline.text() != "Hide Bridge Baseline":
            QMessageBox.warning(self, "Not Available", 
                              "Construction dots are only available when Bridge Baseline is active.\n"
                              "Please click 'Bridge Baseline' button first.")
            self.construction_dots_line.setChecked(False)
            return
        
        # Start construction dots drawing mode
        self.message_text.append("Construction Dots Mode Active")
        self.message_text.append("Click on the graph to add construction dots (P1, P2, etc.)")
        self.message_text.append("Click on the labels (P1, P2) to configure each construction point")
        
        # Enable drawing mode for construction dots
        if self.cid_click is None:
            self.cid_click = self.canvas.mpl_connect('button_press_event', self.on_draw_click)
            self.cid_key = self.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        self.active_line_type = 'construction_dots'
        self.current_points = []
        self.current_artist = None
        self.current_redo_points = []
        
        # Make sure the graph is ready
        self.canvas.draw()
        self.figure.tight_layout()
        # dialog = ConstructionDotsLineDialog(self)
        # if dialog.exec_() == QDialog.Accepted:
        #     config = dialog.get_configuration()
        #     self.message_text.append(f"Construction Dots Config: Spacing: {config['spacing']}, Size: {config['size']}")
            # Add further logic here if needed (e.g., apply construction dots settings)

# =======================================================================================================================================
    def edit_deck_line(self):
        if not self.deck_line.isChecked():
            return
        # dialog = DeckLineDialog(self)
        # if dialog.exec_() == QDialog.Accepted:
        #     config = dialog.get_configuration()
        #     self.message_text.append(f"Deck Line Config: Thickness: {config['thickness']}, Material: {config['material']}")
            # Add further logic here if needed (e.g., apply deck line settings)

# =======================================================================================================================================
# CURVE BUTTON HANDLER
    def on_curve_button_clicked(self, event=None):
        """Handle Curve button and annotation clicks"""
        # Clicked on any curve label → edit
        if event is not None and hasattr(event, 'artist') and event.artist in self.curve_labels:
            self.edit_current_curve()
            return

        # Curve active → ask to complete
        if self.curve_active:
            reply = QMessageBox.question(
                self,
                "Complete Curve",
                "Do you want to complete the current curve segment?\n\n"
                "The curve labels will remain on the graph.",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.Yes
            )
            if reply == QMessageBox.Yes:
                self.complete_current_curve()
            return

        # Start new curve
        self.start_new_curve()

# =======================================================================================================================================
    def start_new_curve(self):
        """Open dialog and start adding curve labels from current point onward"""
        dialog = CurveDialog(self)
        if dialog.exec_() != QDialog.Accepted:
            return

        # Extract full config from dialog
        config = dialog.get_configuration()

        # Validate angle
        if config['angle'] <= 0:
            QMessageBox.warning(self, "Invalid Angle", "Please enter an angle greater than 0.")
            return

        # Get last surface point X
        last_x = self.get_last_surface_x()
        if last_x is None:
            QMessageBox.critical(self, "Error", "Draw at least one point on Surface Line first.")
            return
        
# --------------------------------------------------------
        # NEW: convert chainage → real 3D point (the actual clicked surface point)
        # Find the surface point that has this chainage
        found = False
        for line_type in ['surface']:
            for polyline in reversed(self.line_types[line_type]['polylines']):  # newest first
                for dist, rel_z in reversed(polyline):
                    if abs(dist - last_x) < 0.5:  # small tolerance
                        t = dist / self.total_distance
                        dir_vec = self.zero_end_point - self.zero_start_point
                        pos = self.zero_start_point + t * dir_vec
                        pos[2] = self.zero_start_z + rel_z
                        self.curve_start_point_3d = pos.copy()
                        found = True
                        break
                if found:
                    break
            if found:
                break

        if not found:
            # fallback – just use zero line height
            t = last_x / self.total_distance
            dir_vec = self.zero_end_point - self.zero_start_point
            pos = self.zero_start_point + t * dir_vec
            pos[2] = self.zero_start_z
            self.curve_start_point_3d = pos.copy()
# ---------------------------------------------------------------

        # === START CURVE MODE ===
        self.curve_active = True
        self.curve_start_x = last_x

        # Store config for auto-labeling new points during drawing
        self.current_curve_config = config

        # Clear any previous curve labels
        self.clear_curve_labels()

        # Add first label using the full config from dialog
        self.add_curve_label_at_x(last_x, config)

        # Format display text for button
        outer = config['outer_curve']
        inner = config['inner_curve']
        curve_type = "O&I" if outer and inner else ("O" if outer else ("I" if inner else ""))
        display_text = f"{config['angle']:.1f}° - {curve_type}" if curve_type else f"{config['angle']:.1f}°"

        # Update Curve button appearance
        self.preview_button.setText(f"Curve ({display_text})")
        self.preview_button.setStyleSheet("""
            QPushButton { 
                background-color: #FF5722; 
                color: white; 
                padding: 12px; 
                border-radius: 6px;
                font-size: 15px; 
                font-weight: bold; 
            }
            QPushButton:hover { background-color: #F57C00; }
        """)

        # Log in message area
        chainage = self.get_chainage_label(last_x)
        self.message_text.append(f"Curve started: '{display_text}' at chainage {chainage}")
        self.message_text.append("Every new Surface point will now get this curve label at the top.")
        self.message_text.append("Click any curve label to edit it individually.")
        self.message_text.append("Click 'Curve' button again to finish the segment.")

        self.canvas.draw_idle()

# =======================================================================================================================================
    def add_curve_label_at_x(self, x, config=None):
        """
        Add a curve label at position x.
        config is a dict: {'angle': 5.0, 'outer_curve': True, 'inner_curve': False}
        If None, uses default from current_curve_text
        """
        if config is None:
            # Extract from current text if available
            # Fallback to empty
            angle = 5.0
            outer = False
            inner = False
            if self.current_curve_text:
                try:
                    parts = self.current_curve_text.replace('°', '').split(' - ')
                    angle = float(parts[0])
                    type_part = parts[1] if len(parts) > 1 else ""
                    outer = 'O' in type_part
                    inner = 'I' in type_part
                except:
                    angle = 5.0
                    outer = False
                    inner = False
            config = {'angle': angle, 'outer_curve': outer, 'inner_curve': inner}

        # Format display text
        curve_type = "O&I" if config['outer_curve'] and config['inner_curve'] else \
                     ("O" if config['outer_curve'] else ("I" if config['inner_curve'] else ""))
        display_text = f"{config['angle']:.1f}° - {curve_type}" if curve_type else f"{config['angle']:.1f}°"

        # Create small label like construction dots
        label = self.ax.text(
            x, 1.02,
            display_text,
            transform=self.ax.get_xaxis_transform(),
            ha='center', va='bottom',
            fontsize=9,
            fontweight='bold',
            color='darkred',
            bbox=dict(
                boxstyle="round,pad=0.4",
                facecolor="yellow",
                edgecolor="red",
                linewidth=1.8,
                alpha=0.9
            ),
            zorder=100,
            picker=True
        )

        # Store both artist and its config
        self.curve_labels.append((label, config))

        # Reconnect picker
        if self.curve_pick_id:
            self.canvas.mpl_disconnect(self.curve_pick_id)

        def on_pick(ev):
            # Find which label was clicked
            for artist, cfg in self.curve_labels:
                if ev.artist == artist:
                    self.edit_individual_curve_label(artist, cfg)
                    break

        self.curve_pick_id = self.canvas.mpl_connect('pick_event', on_pick)

# =======================================================================================================================================
    def edit_individual_curve_label(self, label_artist, current_config):
        """Open dialog to edit ONE specific curve label"""
        dialog = CurveDialog(self)

        # Pre-fill with current values
        dialog.angle_edit.setText(f"{current_config['angle']:.1f}")
        dialog.outer_checkbox.setChecked(current_config['outer_curve'])
        dialog.inner_checkbox.setChecked(current_config['inner_curve'])

        if dialog.exec_() != QDialog.Accepted:
            return

        new_config = dialog.get_configuration()
        new_angle = new_config['angle']
        if new_angle <= 0:
            QMessageBox.warning(self, "Invalid Angle", "Angle must be greater than 0.")
            return

        # Format new display text
        outer = new_config['outer_curve']
        inner = new_config['inner_curve']
        curve_type = "O&I" if outer and inner else ("O" if outer else ("I" if inner else ""))
        new_text = f"{new_angle:.1f}° - {curve_type}" if curve_type else f"{new_angle:.1f}°"

        # Update only this label's text
        label_artist.set_text(new_text)

        # Update stored config for this label
        for i, (artist, cfg) in enumerate(self.curve_labels):
            if artist == label_artist:
                self.curve_labels[i] = (artist, new_config)
                break

        self.message_text.append(f"Updated curve label to: {new_text}")
        self.canvas.draw_idle()

# =======================================================================================================================================
    def edit_current_curve(self):
        """Edit curve angle/type – updates all existing labels"""
        if not self.curve_active:
            return

        dialog = CurveDialog(self)
        # Pre-fill current values (you can extract from self.current_curve_text if needed)
        # For simplicity, just open fresh – user can re-enter
        if dialog.exec_() != QDialog.Accepted:
            return

        config = dialog.get_configuration()
        angle = config['angle']
        if angle <= 0:
            QMessageBox.warning(self, "Invalid", "Angle must be > 0")
            return

        outer = config['outer_curve']
        inner = config['inner_curve']
        curve_type = "O&I" if outer and inner else ("O" if outer else ("I" if inner else ""))
        new_text = f"{angle:.1f}° - {curve_type}" if curve_type else f"{angle:.1f}°"

        # Update stored text
        self.current_curve_text = new_text

        # Update ALL existing curve labels
        for label in self.curve_labels:
            label.set_text(new_text)

        # Update button
        self.preview_button.setText(f"Curve ({new_text})")

        self.message_text.append(f"Curve updated to: {new_text}")
        self.canvas.draw_idle()

# =======================================================================================================================================
    def complete_current_curve(self):
        """End curve mode – keep all labels"""
        if not self.curve_active:
            return

        self.curve_active = False
        self.curve_start_x = None

        # Reset button
        self.preview_button.setText("Curve")
        self.preview_button.setStyleSheet("""
            QPushButton { background-color: #808080; color: white; padding: 12px; border-radius: 6px;
                          font-size: 15px; font-weight: bold; }
            QPushButton:hover { background-color: #6E6E6E; }
        """)

        self.message_text.append(f"Curve completed! {len(self.curve_labels)} labels remain on graph.")
        self.canvas.draw_idle()

# =======================================================================================================================================
    def clear_curve_labels(self):
        """Remove all curve labels (used on new curve or reset)"""
        for label in self.curve_labels:
            try:
                label.remove()
            except:
                pass
        self.curve_labels = []

        if self.curve_pick_id:
            self.canvas.mpl_disconnect(self.curve_pick_id)
            self.curve_pick_id = None

# =======================================================================================================================================
    def get_last_surface_x(self):
        """
        Returns the X-coordinate (distance along zero line) of the most recent point
        on the Surface Line (either from completed polylines or current drawing).
        Returns None if no surface points exist.
        """
        # Check completed surface polylines (in reverse order - last one first)
        for polyline in reversed(self.line_types['surface']['polylines']):
            if polyline:  # if not empty
                return polyline[-1][0]  # return X of last point

        # Check if currently drawing a surface line
        if self.active_line_type == 'surface' and self.current_points:
            if len(self.current_points) > 0:
                return self.current_points[-1][0]

        # No surface points found
        return None

# =======================================================================================================================================
    def undo_graph(self):
        if self.active_line_type is None:
            return
    
        if self.active_line_type == 'construction_dots' and self.current_points:
            # For construction dots, remove the last point only
            if len(self.current_points) > 0:
                popped = self.current_points.pop()
                self.current_redo_points.append(popped)
                
                # Remove the last point label if it exists
                if self.current_point_labels:
                    label = self.current_point_labels.pop()
                    if label and label in self.ax.texts:
                        label.remove()
                
                # Remove the last visual dot if it exists
                if hasattr(self, 'construction_dot_artists') and self.construction_dot_artists:
                    artist = self.construction_dot_artists.pop()
                    try:
                        artist.remove()
                    except:
                        pass
                
                self.canvas.draw()
                self.figure.tight_layout()
            return
            
        # Original logic for other line types
        if self.current_points:
            if len(self.current_points) > 0:
                popped = self.current_points.pop()
                self.current_redo_points.append(popped)
                color = self.line_types[self.active_line_type]['color']
                
                # No labels to remove for other line types
                
                if self.current_points:
                    xs = [p[0] for p in self.current_points]
                    ys = [p[1] for p in self.current_points]
                    if self.current_artist is None:
                        self.current_artist, = self.ax.plot(xs, ys, color=color, linewidth=2, marker='o', markersize=5)
                    else:
                        self.current_artist.set_data(xs, ys)
                else:
                    if self.current_artist is not None:
                        self.current_artist.remove()
                        self.current_artist = None
                
                self.canvas.draw()
                self.figure.tight_layout()
        elif self.all_graph_lines:
            lt, points, artist, ann = self.all_graph_lines.pop()
            artist.remove()
            if ann:  # Only remove annotation if it exists
                ann.remove()
            self.line_types[lt]['artists'].pop()
            self.line_types[lt]['polylines'].pop()
            self.redo_stack.append((lt, points))
            
            # Remove any stored point labels for this polyline (only for construction dots)
            if lt == 'construction_dots' and self.point_labels:
                # Remove the last n labels where n is the number of points
                for _ in range(len(points)):
                    if self.point_labels:
                        label = self.point_labels.pop()
                        if label and label in self.ax.texts:
                            label.remove()
            
            self.canvas.draw()
            self.figure.tight_layout()

# -------------------------------------------------
# REDO GRAPH (Modified)
# -------------------------------------------------
    def redo_graph(self):
        if self.active_line_type is None:
            return
        
        if self.current_redo_points:
            if len(self.current_redo_points) > 0:
                point = self.current_redo_points.pop()
                self.current_points.append(point)
                color = self.line_types[self.active_line_type]['color']
                xs = [p[0] for p in self.current_points]
                ys = [p[1] for p in self.current_points]
                
                # Add point label only for construction dots
                if self.active_line_type == 'construction_dots':
                    label = self.add_point_label(point[0], point[1], len(self.current_points), self.active_line_type)
                    if label:  # Only append if label was created
                        self.current_point_labels.append(label)
                
                if self.current_artist is None:
                    self.current_artist, = self.ax.plot(xs, ys, color=color, linewidth=2, marker='o', markersize=5)
                else:
                    self.current_artist.set_data(xs, ys)
                
                self.canvas.draw()
                self.figure.tight_layout()
        elif self.redo_stack:
            lt, points = self.redo_stack.pop()
            color = self.line_types[lt]['color']
            xs = [p[0] for p in points]
            ys = [p[1] for p in points]
            new_artist, = self.ax.plot(xs, ys, color=color, linewidth=2, marker='o', markersize=5)
            
            # Re-add point labels only for construction dots
            if lt == 'construction_dots':
                for i, (x, y) in enumerate(points, 1):
                    label = self.add_point_label(x, y, i, lt)
                    if label:  # Only append if label was created
                        self.point_labels.append(label)
            
            # Calculate total length in meters
            length = 0.0
            for i in range(1, len(points)):
                dx = points[i][0] - points[i-1][0]
                dy = points[i][1] - points[i-1][1]
                length += np.sqrt(dx**2 + dy**2)
            
            # Annotate the length at the end point (only for non-construction dots)
            if lt != 'construction_dots':
                end_x, end_y = points[-1]
                ann = self.ax.annotate(f'{length:.2f}m', xy=(end_x, end_y), xytext=(5, 5),
                                       textcoords='offset points',
                                       bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.7),
                                       arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
            else:
                ann = None
            
            self.all_graph_lines.append((lt, points, new_artist, ann))
            self.line_types[lt]['artists'].append(new_artist)
            self.line_types[lt]['polylines'].append(points[:])
            
            self.canvas.draw()
            self.figure.tight_layout()

# -------------------------------------------------
# FINISH CURRENT POLYLINE (Modified)
# -------------------------------------------------
    def finish_current_polyline(self):
        if self.active_line_type == 'construction_dots':
            # For construction dots - store as individual points without connecting line
            if len(self.current_points) > 0:
                # Create individual point markers for each point
                color = self.line_types[self.active_line_type]['color']
                
                # Plot all points as separate markers
                xs = [p[0] for p in self.current_points]
                ys = [p[1] for p in self.current_points]
                
                # Create a single scatter plot for all construction dots
                # This makes it easier to manage as a single artist
                scatter_artist = self.ax.scatter(xs, ys, color=color, s=100, marker='o', zorder=5)
                
                self.line_types[self.active_line_type]['artists'].append(scatter_artist)
                self.line_types[self.active_line_type]['polylines'].append(self.current_points[:])

                # Add curve labels to all points in this completed polyline if curve is active
                if self.active_line_type == 'surface' and self.curve_active and self.current_curve_text:
                    for px, py in self.current_points:
                        self.add_curve_label_at_x(px)
                

                for label in self.current_point_labels:
                    if label:
                        self.point_labels.append(label)
                
                self.all_graph_lines.append((self.active_line_type, self.current_points[:], scatter_artist, None))
                
                self.message_text.append(f"Construction dots completed with {len(self.current_points)} points")
                
                self.current_points = []
                self.current_artist = None
                self.current_redo_points = []
                self.current_point_labels = []
                
                # Clear temporary construction dot artists
                if hasattr(self, 'construction_dot_artists'):
                    for artist in self.construction_dot_artists:
                        try:
                            artist.remove()
                        except:
                            pass
                    self.construction_dot_artists = []
                
                self.canvas.draw()
                self.figure.tight_layout()
            elif len(self.current_points) == 1:
                self.message_text.append("Need at least 1 point for construction dots")
            return
        
        # For other line types (original code remains the same)
        if len(self.current_points) > 1 and self.active_line_type:
            xs = [p[0] for p in self.current_points]
            ys = [p[1] for p in self.current_points]
            color = self.line_types[self.active_line_type]['color']
            line_artist, = self.ax.plot(xs, ys, color=color, linewidth=2, marker='o', markersize=5)
            
            self.line_types[self.active_line_type]['artists'].append(line_artist)
            self.line_types[self.active_line_type]['polylines'].append(self.current_points[:])
            

            # Save the polyline to the current mode's data
            if self.current_mode == 'road' and self.active_line_type in ['construction', 'surface', 'road_surface', 'zero']:
                if self.active_line_type not in self.road_lines_data:
                    self.road_lines_data[self.active_line_type] = {'polylines': [], 'artists': []}
                self.road_lines_data[self.active_line_type]['polylines'].append(self.current_points.copy())
                
            elif self.current_mode == 'bridge' and self.active_line_type in ['deck_line', 'projection_line', 'construction_dots', 'zero']:
                if self.active_line_type not in self.bridge_lines_data:
                    self.bridge_lines_data[self.active_line_type] = {'polylines': [], 'artists': []}
                self.bridge_lines_data[self.active_line_type]['polylines'].append(self.current_points.copy())
            
            length = 0.0
            for i in range(1, len(self.current_points)):
                dx = self.current_points[i][0] - self.current_points[i-1][0]
                dy = self.current_points[i][1] - self.current_points[i-1][1]
                length += np.sqrt(dx**2 + dy**2)
            
            if self.active_line_type != 'construction_dots':
                end_x, end_y = self.current_points[-1]
                ann = self.ax.annotate(f'{length:.2f}m', xy=(end_x, end_y), xytext=(5, 5),
                                    textcoords='offset points',
                                    bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.7),
                                    arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
            else:
                ann = None
            
            self.all_graph_lines.append((self.active_line_type, self.current_points[:], line_artist, ann))
            self.current_redo_points = []
            self.current_point_labels = []
            
            self.canvas.draw()
            self.figure.tight_layout()
        
        self.current_points = []
        if self.current_artist is not None:
            self.current_artist.remove()
            self.current_artist = None
        self.message_text.append(f"{self.active_line_type.replace('_', ' ').title()} completed")
    
# -------------------------------------------------
# ADD POINT LABEL (New method)
# -------------------------------------------------
    def add_point_label(self, x, y, point_number, line_type):
        """Add a small label above the clicked point on the graph - ONLY for construction dots"""
        # Only create labels for construction dots
        if line_type != 'construction_dots':
            return None
        
        # Create a small label (like P1, P2, etc.)
        label = f'P{point_number}'
        
        # For construction dots, place at the TOP of the graph area
        label_x = x
        label_y = 1.05  # 5% above the top of the graph area
        
        # Make construction dots labels clickable with distinctive appearance
        text_obj = self.ax.text(
            label_x, label_y, label, 
            transform=self.ax.get_xaxis_transform(),  # x in data coords, y in axes coords
            fontsize=10, 
            color='red',
            fontweight='bold',
            ha='center', 
            va='center',
            bbox=dict(
                boxstyle='round,pad=0.5', 
                facecolor='yellow', 
                alpha=0.9,
                edgecolor='red',
                linewidth=2
            ),
            picker=True  # Make it clickable - THIS IS IMPORTANT!
        )
        
        # Store point data for click handling
        text_obj.point_data = {
            'type': 'construction_dot',
            'point_number': point_number,
            'x': x,
            'y': y,
            'line_type': line_type
        }
        
        return text_obj
    
# -------------------------------------------------
# ON DRAW CLICK (Modified)
# -------------------------------------------------
    def on_draw_click(self, event):
        if event.inaxes != self.ax or self.active_line_type is None:
            return
        x, y = event.xdata, event.ydata
        if x is None or y is None:
            return

        # Get current time for double-click detection
        current_time = time.time()
        time_diff = current_time - self.last_click_time

        # Handle double-click (for line completion)
        if time_diff < self.double_click_threshold and len(self.current_points) > 1:
            self.finish_current_polyline()
            self.last_click_time = 0
            return

        # For construction dots - separate handling (your existing code)
        if self.active_line_type == 'construction_dots':
            if len(self.current_points) == 0:
                self.current_points.append((x, y))
                label = self.add_point_label(x, y, len(self.current_points), self.active_line_type)
                if label:
                    self.current_point_labels.append(label)

                color = self.line_types[self.active_line_type]['color']
                self.current_artist, = self.ax.plot([x], [y], color=color, marker='o', markersize=8, linestyle='')
            else:
                self.current_points.append((x, y))
                label = self.add_point_label(x, y, len(self.current_points), self.active_line_type)
                if label:
                    self.current_point_labels.append(label)

                color = self.line_types[self.active_line_type]['color']
                new_artist, = self.ax.plot([x], [y], color=color, marker='o', markersize=8, linestyle='')

                if not hasattr(self, 'construction_dot_artists'):
                    self.construction_dot_artists = []
                self.construction_dot_artists.append(new_artist)

            self.last_click_time = current_time
            self.canvas.draw_idle()
            return

        # For other line types (including surface)
        if len(self.current_points) == 0:
            self.current_points.append((x, y))
        else:
            self.current_points.append((x, y))

        # Update current artist with markers
        color = self.line_types[self.active_line_type]['color']
        xs = [p[0] for p in self.current_points]
        ys = [p[1] for p in self.current_points]

        if self.current_artist is None:
            self.current_artist, = self.ax.plot(xs, ys, color=color, linewidth=2, marker='o', markersize=5)
        else:
            self.current_artist.set_data(xs, ys)
            self.current_artist.set_color(color)

        # =====================================================
        # SPECIAL: Add curve label when drawing Surface Line
        # =====================================================
        if self.active_line_type == 'surface' and self.curve_active:
            latest_x = self.current_points[-1][0]  # X of the newly added point
            self.add_curve_label_at_x(latest_x)

        self.last_click_time = current_time
        self.canvas.draw_idle()

# -------------------------------------------------
# Graph plot (placeholder for future use) - now embedded in canvas
# -------------------------------------------------
    def plot_graph(self, x, y):
        if self.ax:
            self.ax.clear()
            self.ax.plot(x, y)
            self.ax.grid(True)
            self.ax.set_title("Sample Graph")
            self.ax.set_xlabel("X-axis")
            self.ax.set_ylabel("Y-axis")
            self.canvas.draw()
            self.figure.tight_layout()
        else:
            self.figure.clear()
            ax = self.figure.add_subplot(111)
            ax.plot(x, y)
            ax.set_title("Sample Graph")
            ax.set_xlabel("X-axis")
            ax.set_ylabel("Y-axis")
            ax.grid(True)
            self.canvas.draw()
            self.figure.tight_layout()

# =======================================================================================================================================
    def preview_lines_on_3d(self):
        if not self.zero_line_set:
            self.message_text.append("Zero line must be set before previewing.")
            return
        has_polylines = any(self.line_types[lt]['polylines'] for lt in ['surface', 'construction', 'road_surface'])
        if not has_polylines:
            self.message_text.append("No lines drawn to preview.")
            return

        # # Clear road planes when previewing to avoid clutter
        # self.clear_road_planes()

        # Clear previous preview actors
        for actor in self.preview_actors:
            self.renderer.RemoveActor(actor)
        self.preview_actors = []

        dir_vec = self.zero_end_point - self.zero_start_point
        p1_z = self.zero_start_z

        for line_type in ['surface', 'construction', 'road_surface']:
            if self.line_types[line_type]['polylines']:
                color = self.line_types[line_type]['color']
                for poly_2d in self.line_types[line_type]['polylines']:
                    points_3d = []
                    for dist, rel_z in poly_2d:
                        t = dist / self.total_distance
                        pos_along = self.zero_start_point + t * dir_vec
                        z_abs = p1_z + rel_z
                        pos_3d = np.array([pos_along[0], pos_along[1], z_abs])
                        points_3d.append(pos_3d)
                    for i in range(len(points_3d) - 1):
                        self.add_preview_line(points_3d[i], points_3d[i + 1], color)
        self.vtk_widget.GetRenderWindow().Render()
        self.message_text.append("Preview lines mapped on 3D point cloud.")

    def add_preview_line(self, p1, p2, color):
        line = vtkLineSource()
        line.SetPoint1(p1[0], p1[1], p1[2])
        line.SetPoint2(p2[0], p2[1], p2[2])
        mapper = vtkPolyDataMapper()
        mapper.SetInputConnection(line.GetOutputPort())
        actor = vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(self.colors.GetColor3d(color))
        actor.GetProperty().SetLineWidth(3)
        self.renderer.AddActor(actor)
        self.preview_actors.append(actor)

# =======================================================================================================================================
# MAP ROAD BASELINES TO 3D PLANES
# =======================================================================================================================================
    def map_baselines_to_3d_planes(self):
        """
        Main function: Called when user clicks 'Map on 3D' button.
        Creates planes for ALL currently drawn baseline types.
        Planes are ADDED incrementally — previous planes remain visible.
        Each time user draws a new line and clicks 'Map on 3D', new planes are added on top.
        """
        if not self.zero_line_set:
            QMessageBox.warning(self, "Zero Line Required", "Please set the Zero Line first before mapping baselines to planes.")
            self.message_text.append("Zero line must be set.")
            return

        # Collect all current polylines from baseline types
        current_polylines = {}
        new_segment_count = 0
        for ltype in self.baseline_types:
            polylines = self.line_types[ltype]['polylines']
            if polylines:
                current_polylines[ltype] = polylines
                new_segment_count += sum(len(poly) - 1 for poly in polylines if len(poly) >= 2)

        if new_segment_count == 0:
            QMessageBox.information(self, "No New Lines", "No new baseline segments detected since last mapping.\nDraw or modify lines and try again.")
            self.message_text.append("No new baseline segments to map.")
            return

        # Open dialog for width (same width for all this time)
        dialog = RoadPlaneWidthDialog(self)
        if dialog.exec_() != QDialog.Accepted:
            return

        width = dialog.get_width()
        half_width = width / 2.0

        # DO NOT clear previous planes — we want to accumulate!
        # self.clear_baseline_planes()  ← REMOVED INTENTIONALLY

        zero_dir_vec = self.zero_end_point - self.zero_start_point
        zero_length = self.total_distance
        ref_z = self.zero_start_z

        plane_count_this_time = 0

        for ltype, polylines in current_polylines.items():
            rgba = self.plane_colors.get(ltype, (0.5, 0.5, 0.5, 0.4))
            color_rgb = rgba[:3]
            opacity = rgba[3]

            for poly_2d in polylines:
                if len(poly_2d) < 2:
                    continue

                for i in range(len(poly_2d) - 1):
                    dist1, rel_z1 = poly_2d[i]
                    dist2, rel_z2 = poly_2d[i + 1]

                    pos1 = self.zero_start_point + (dist1 / zero_length) * zero_dir_vec
                    pos2 = self.zero_start_point + (dist2 / zero_length) * zero_dir_vec

                    z1 = ref_z + rel_z1
                    z2 = ref_z + rel_z2

                    center1 = np.array([pos1[0], pos1[1], z1])
                    center2 = np.array([pos2[0], pos2[1], z2])

                    seg_dir = center2 - center1
                    seg_len = np.linalg.norm(seg_dir)
                    if seg_len < 1e-6:
                        continue
                    seg_unit = seg_dir / seg_len

                    # Compute horizontal perpendicular direction
                    horiz = np.array([seg_unit[0], seg_unit[1], 0.0])
                    hlen = np.linalg.norm(horiz)
                    if hlen < 1e-6:
                        # Fallback: perpendicular to zero line
                        zero_unit = zero_dir_vec / np.linalg.norm(zero_dir_vec)
                        perp = np.array([-zero_unit[1], zero_unit[0], 0.0])
                    else:
                        horiz /= hlen
                        perp = np.array([-horiz[1], horiz[0], 0.0])

                    # Normalize perp just in case
                    perp_len = np.linalg.norm(perp)
                    if perp_len > 0:
                        perp /= perp_len

                    # Four corners of the plane segment
                    c1 = center1 + perp * half_width
                    c2 = center1 - perp * half_width
                    c3 = center2 - perp * half_width
                    c4 = center2 + perp * half_width

                    # Create rectangular plane
                    plane = vtkPlaneSource()
                    plane.SetOrigin(c1[0], c1[1], c1[2])
                    plane.SetPoint1(c4[0], c4[1], c4[2])
                    plane.SetPoint2(c2[0], c2[1], c2[2])
                    plane.SetXResolution(12)
                    plane.SetYResolution(2)
                    plane.Update()

                    mapper = vtkPolyDataMapper()
                    mapper.SetInputConnection(plane.GetOutputPort())

                    actor = vtkActor()
                    actor.SetMapper(mapper)
                    actor.GetProperty().SetColor(*color_rgb)
                    actor.GetProperty().SetOpacity(opacity)
                    actor.GetProperty().EdgeVisibilityOn()
                    actor.GetProperty().SetEdgeColor(*color_rgb)
                    actor.GetProperty().SetLineWidth(1.5)

                    # Add to scene and store
                    self.renderer.AddActor(actor)
                    self.baseline_plane_actors.append(actor)
                    plane_count_this_time += 1

        # Final render
        self.vtk_widget.GetRenderWindow().Render()

        # Feedback
        total_planes = len(self.baseline_plane_actors)
        self.message_text.append(f"Added {plane_count_this_time} new plane segments (width: {width:.2f}m). Total visible: {total_planes}")
        
        QMessageBox.information(
            self,
            "Planes Added Successfully",
            f"Added {plane_count_this_time} new plane segments in this mapping.\n"
            f"Width: {width:.2f} m (±{half_width:.2f} m each side)\n\n"
            f"Total planes now visible: {total_planes}\n\n"
            "You can continue drawing more lines and click 'Map on 3D' again — "
            "new planes will be added without removing old ones.\n\n"
            "Color Guide:\n"
            "• Surface → Green\n"
            "• Construction → Red\n"
            "• Road Surface → Blue\n"
            "• Deck Line → Orange\n"
            "• Projection → Purple\n"
            "• Material → Yellow"
        )

    def clear_baseline_planes(self):
        """Remove ALL accumulated baseline plane actors — used only on reset or new worksheet."""
        for actor in self.baseline_plane_actors:
            # Remove actor from renderer without checking for renderer property
            self.renderer.RemoveActor(actor)
        self.baseline_plane_actors.clear()
        if hasattr(self, 'vtk_widget'):
            self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================
    def reset_zero_drawing(self):
        for actor in self.temp_zero_actors:
            self.renderer.RemoveActor(actor)
            if actor in self.measurement_actors:
                self.measurement_actors.remove(actor)
        self.temp_zero_actors = []
        self.zero_points = []
        
        # Reset scale to default
        self.scale_ax.set_xticks(np.arange(0, self.total_distance + 1, 5))
        self.scale_ax.set_xticklabels([f"{x:.0f}" for x in np.arange(0, self.total_distance + 1, 5)])
        self.scale_canvas.draw()

# =======================================================================================================================================
    def focus_camera_on_full_cloud(self):
        """Focus camera on the entire point cloud"""
        if not self.point_cloud or not self.point_cloud_actor:
            return
        try:
            bounds = self.point_cloud_actor.GetBounds()
            if bounds[0] >= bounds[1]:  # Invalid bounds
                return
            center = [(bounds[0] + bounds[1]) / 2,
                      (bounds[2] + bounds[3]) / 2,
                      (bounds[4] + bounds[5]) / 2]
            size_x = bounds[1] - bounds[0]
            size_y = bounds[3] - bounds[2]
            size_z = bounds[5] - bounds[4]
            avg_size = (size_x + size_y + size_z) / 3.0

            camera = self.renderer.GetActiveCamera()
            camera.SetFocalPoint(*center)
            offset = avg_size * 1.8  # Slightly farther for full view
            camera.SetPosition(center[0] + offset, center[1], center[2] + offset * 0.2)
            camera.SetViewUp(0, 0, 1)
            camera.SetViewAngle(15.0)
            self.renderer.ResetCameraClippingRange()
            self.vtk_widget.GetRenderWindow().Render()
        except Exception as e:
            print(f"Error in full cloud focus: {e}")

# =======================================================================================================================================
    def update_camera_view(self, slider_value):
        """Update camera to always look at the current slider marker (red sphere) from a fixed side view."""
        if not self.zero_line_set or not self.point_cloud:
            self.focus_camera_on_full_cloud()
            return

        try:
            # Calculate current position along the zero line
            fraction = slider_value / 100.0
            current_dist = fraction * self.total_distance

            dir_vec = self.zero_end_point - self.zero_start_point
            unit_dir = dir_vec / np.linalg.norm(dir_vec)

            # World position of the marker (on the zero line at reference elevation)
            pos_along = self.zero_start_point + fraction * dir_vec
            marker_pos = np.array([pos_along[0], pos_along[1], self.zero_start_z])

            # Add or update the red sphere marker
            self.add_or_update_slider_marker(marker_pos)

            # Define camera distance and offset (side view, slightly elevated)
            camera_distance = max(self.total_distance * 0.6, 30.0)  # Scale with project size, min 30m
            elevation_offset = camera_distance * 0.2  # Slight upward angle

            # Camera position: offset perpendicular to the zero line direction (to the right side)
            # Compute horizontal perpendicular vector (rotate zero direction 90° clockwise in XY)
            zero_horizontal = np.array([unit_dir[0], unit_dir[1], 0.0])
            h_len = np.linalg.norm(zero_horizontal)
            if h_len < 1e-6:
                perp = np.array([0.0, 1.0, 0.0])
            else:
                perp = np.array([-zero_horizontal[1], zero_horizontal[0], 0.0]) / h_len

            camera_pos = marker_pos + perp * camera_distance
            camera_pos[2] += elevation_offset  # Lift camera a bit for better view

            # Set camera
            camera = self.renderer.GetActiveCamera()
            camera.SetFocalPoint(marker_pos[0], marker_pos[1], marker_pos[2])
            camera.SetPosition(camera_pos[0], camera_pos[1], camera_pos[2])
            camera.SetViewUp(0.0, 0.0, 1.0)  # Keep up vector as +Z
            camera.SetViewAngle(10.0)       # Reasonable field of view

            # Optional: slightly tighter clipping for cleaner view
            self.renderer.ResetCameraClippingRange()

            self.vtk_widget.GetRenderWindow().Render()

        except Exception as e:
            print(f"Error updating camera view: {e}")
            self.focus_camera_on_full_cloud()

# =======================================================================================================================================
    def volume_changed(self, value):
        """Main handler when volume slider moves – keeps marker and camera perfectly aligned."""
        print(f"Volume slider changed to: {value}%")

        if self.zero_line_set:
            # Update scale marker and chainage label
            self.update_scale_marker()

            # Update main graph vertical marker
            self.update_main_graph_marker(value)

            # **CRITICAL**: Update camera to follow the red sphere marker exactly
            self.update_camera_view(value)

        else:
            # No zero line → show full cloud and remove marker
            self.focus_camera_on_full_cloud()
            self.remove_slider_marker()

        # Always scroll the 2D graph horizontally
        self.scroll_graph_with_slider(value)

# =======================================================================================================================================
    def add_or_update_slider_marker(self, world_pos):
        """Create or move the red sphere that marks the current slider/chainage position"""
        if self.slider_marker_actor is None:
            # Create new sphere
            sphere = vtkSphereSource()
            sphere.SetRadius(self.slider_marker_radius)
            sphere.SetThetaResolution(32)
            sphere.SetPhiResolution(32)

            mapper = vtkPolyDataMapper()
            mapper.SetInputConnection(sphere.GetOutputPort())

            self.slider_marker_actor = vtkActor()
            self.slider_marker_actor.SetMapper(mapper)
            self.slider_marker_actor.GetProperty().SetColor(1.0, 0.0, 0.0)  # Red
            self.slider_marker_actor.GetProperty().SetOpacity(0.9)

            self.renderer.AddActor(self.slider_marker_actor)

        # Always update position
        self.slider_marker_actor.SetPosition(world_pos[0], world_pos[1], world_pos[2])
        self.vtk_widget.GetRenderWindow().Render()

    def remove_slider_marker(self):
        """Remove the slider position sphere from the scene"""
        if self.slider_marker_actor is not None:
            self.renderer.RemoveActor(self.slider_marker_actor)
            self.slider_marker_actor = None
            self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================
    def save_current_design_layer(self):
        """Save all currently drawn and checked baselines as JSON files in the current design layer folder."""
        if not hasattr(self, 'current_worksheet_name') or not self.current_worksheet_name:
            QMessageBox.warning(self, "No Worksheet", "No active worksheet found. Create or open a worksheet first.")
            return

        if not hasattr(self, 'current_layer_name') or not self.current_layer_name:
            QMessageBox.warning(self, "No Layer", "No active design layer. Please create a new design layer first.")
            return

        # Get the design layer folder
        layer_folder = os.path.join(
            self.WORKSHEETS_BASE_DIR,
            self.current_worksheet_name,
            "designs",
            self.current_layer_name
        )

        if not os.path.exists(layer_folder):
            QMessageBox.critical(self, "Folder Missing", f"Design layer folder not found:\n{layer_folder}")
            return

        if not self.zero_line_set:
            QMessageBox.warning(self, "Zero Line Required", "Zero line must be set before saving baselines.")
            return

        # Track what was saved
        saved_count = 0
        saved_files = []

        # Direction vector and reference
        dir_vec = self.zero_end_point - self.zero_start_point
        zero_length = self.total_distance
        ref_z = self.zero_start_z

        # Last used width (we'll use the most recent one from mapping; fallback to 10.0)
        last_width = getattr(self, 'last_plane_width', 10.0)

        # Define baseline types to save (only if checkbox is checked)
        baseline_checkboxes = {
            'surface': self.surface_baseline,
            'construction': self.construction_line,
            'road_surface': self.road_surface_line,
            'deck_line': self.deck_line,
            'projection_line': self.projection_line,
        }

        for ltype, checkbox in baseline_checkboxes.items():
            if not checkbox.isChecked():
                continue

            polylines = self.line_types[ltype]['polylines']
            if not polylines:
                continue

            # Prepare data structure
            baseline_data = {
                "baseline_type": ltype.replace('_', ' ').title(),
                "baseline_key": ltype,
                "color": self.line_types[ltype]['color'],
                "width_meters": last_width,
                "zero_line_start": self.zero_start_point.tolist(),
                "zero_line_end": self.zero_end_point.tolist(),
                "zero_start_elevation": float(ref_z),
                "total_chainage_length": float(zero_length),
                "polylines": []
            }

            # Convert each polyline
            for poly_2d in polylines:
                poly_3d_points = []
                for dist, rel_z in poly_2d:
                    t = dist / zero_length
                    pos_along = self.zero_start_point + t * dir_vec
                    abs_z = ref_z + rel_z
                    world_point = [float(pos_along[0]), float(pos_along[1]), float(abs_z)]

                    poly_3d_points.append({
                        "chainage_m": float(dist),
                        "relative_elevation_m": float(rel_z),
                        "world_coordinates": world_point
                    })

                if len(poly_3d_points) >= 2:
                    start_chainage = poly_3d_points[0]["chainage_m"]
                    end_chainage = poly_3d_points[-1]["chainage_m"]
                    baseline_data["polylines"].append({
                        "start_chainage_m": float(start_chainage),
                        "end_chainage_m": float(end_chainage),
                        "points": poly_3d_points
                    })

            if not baseline_data["polylines"]:
                continue

            # Save to JSON file named after baseline type
            json_filename = f"{ltype}_baseline.json"
            json_path = os.path.join(layer_folder, json_filename)

            try:
                with open(json_path, 'w', encoding='utf-8') as f:
                    json.dump(baseline_data, f, indent=4, ensure_ascii=False)

                saved_count += 1
                saved_files.append(json_filename)

            except Exception as e:
                QMessageBox.critical(self, "Save Failed", f"Could not save {json_filename}:\n{str(e)}")
                self.message_text.append(f"Error saving {ltype}: {str(e)}")
                return

        # Final feedback
        if saved_count > 0:
            file_list = "\n".join([f"• {f}" for f in saved_files])
            self.message_text.append(f"Saved {saved_count} baseline(s) to design layer '{self.current_layer_name}':")
            self.message_text.append(file_list)
            self.message_text.append(f"Location: {layer_folder}")

            QMessageBox.information(
                self,
                "Save Successful",
                f"Successfully saved {saved_count} baseline(s):\n\n{file_list}\n\n"
                f"Folder:\n{layer_folder}\n\n"
                "You can now close or continue editing."
            )
        else:
            QMessageBox.information(self, "Nothing to Save", "No checked baselines with drawn lines found to save.")
            self.message_text.append("No baselines saved (none checked or drawn).")

# =======================================================================================================================================
    def get_current_design_layer_path(self):
        """Return path to current active design layer, fallback to worksheet root"""
        if not hasattr(self, 'current_worksheet_name') or not self.current_worksheet_name:
            self.message_text.append("No active worksheet — cannot save zero line config.")
            return None
        worksheet_path = os.path.join(self.WORKSHEETS_BASE_DIR, self.current_worksheet_name)
        if hasattr(self, 'current_layer_name') and self.current_layer_name:
            layer_path = os.path.join(worksheet_path, "designs", self.current_layer_name)
            if os.path.exists(layer_path):
                return layer_path
            else:
                self.message_text.append(f"Design layer folder not found: {layer_path}")
                self.message_text.append("Saving zero_line_config.json in worksheet root.")
        return worksheet_path

# =======================================================================================================================================
    def save_zero_line_config_to_current_layer(self):
        """Save zero line configuration as JSON in current design layer or worksheet root"""
        save_path = self.get_current_design_layer_path()
        if not save_path:
            return False

        zero_config = {
            "zero_line_set": True,
            "point1": {
                "coordinates": self.zero_start_point.tolist(),
                "km": self.zero_start_km if (hasattr(self, 'zero_start_km') and self.zero_start_km is not None) else None,
                "chainage_m": 0.0
            },
            "point2": {
                "coordinates": self.zero_end_point.tolist(),
                "km": self.zero_end_km if (hasattr(self, 'zero_end_km') and self.zero_end_km is not None) else None,
                "chainage_m": float(self.total_distance)
            },
            "total_length_m": float(self.total_distance),
            "interval_m": float(self.zero_interval),
            "reference_elevation_z": float(self.zero_start_z),
            "saved_at": datetime.now().isoformat(),
            "saved_by": getattr(self, 'current_user', 'unknown')
        }

        json_path = os.path.join(save_path, "zero_line_config.json")
        try:
            os.makedirs(save_path, exist_ok=True)
            with open(json_path, 'w', encoding='utf-8') as f:
                json.dump(zero_config, f, indent=4, ensure_ascii=False)
            self.message_text.append("Zero line configuration saved to:")
            self.message_text.append(f"   {json_path}")
            return True
        except Exception as e:
            error_msg = f"Failed to save zero_line_config.json: {str(e)}"
            self.message_text.append(error_msg)
            QMessageBox.critical(self, "Save Failed", error_msg)
            return False
   
# =======================================================================================================================================
    def open_zero_line_dialog(self, auto_opened=False):
        """Open Zero Line Configuration Dialog — called manually or automatically"""
        # Pass current values if already set
        p1 = self.zero_start_point if self.zero_line_set else None
        p2 = self.zero_end_point if self.zero_line_set else None

        km1 = getattr(self, 'zero_start_km', None)
        km2 = getattr(self, 'zero_end_km', None)
        interval = getattr(self, 'zero_interval', 20.0)

        dialog = ZeroLineDialog(
            point1=p1, point2=p2,
            km1=km1, km2=km2,
            interval=interval,
            parent=self
        )

        if dialog.exec_() == QDialog.Accepted:
            p1, p2 = dialog.get_points()
            if p1 is None or p2 is None:
                QMessageBox.warning(self, "Invalid Input", "Please enter valid numeric coordinates.")
                if auto_opened:
                    self.zero_line.setChecked(False)
                return

            try:
                km1_text = dialog.km1_edit.text().strip()
                km2_text = dialog.km2_edit.text().strip()
                interval_text = dialog.interval_edit.text().strip()

                self.zero_start_km = int(km1_text) if km1_text else None
                self.zero_end_km = int(km2_text) if km2_text else None
                self.zero_interval = float(interval_text) if interval_text else 20.0
            except ValueError:
                QMessageBox.warning(self, "Invalid Input", "KM must be integer, interval must be a number.")
                if auto_opened:
                    self.zero_line.setChecked(False)
                return

            # Update zero line state
            self.zero_start_point = p1
            self.zero_end_point = p2
            self.zero_start_z = p1[2]  # Reference elevation from Point 1 Z
            self.zero_line_set = True

            dir_vec = p2 - p1
            self.total_distance = np.linalg.norm(dir_vec)
            self.original_total_distance = self.total_distance

            # Update graph axis and ticks
            self.update_chainage_ticks()
            if hasattr(self, 'scale_section'):
                self.scale_section.setVisible(True)

            # Redraw zero line on 2D graph
            if hasattr(self, 'zero_graph_line') and self.zero_graph_line:
                self.zero_graph_line.remove()
            self.zero_graph_line, = self.ax.plot([0, self.total_distance], [0, 0],
                                                color='purple', linewidth=3, label='Zero Line')
            self.canvas.draw()

            # Refresh 3D view
            if hasattr(self, 'vtk_widget'):
                self.vtk_widget.GetRenderWindow().Render()

            # === CRITICAL: Save zero line config to current design layer ===
            success = self.save_zero_line_config_to_current_layer()
            if success:
                self.message_text.append("Zero line successfully configured and saved to current layer!")

            # User feedback
            feedback = [
                "Zero Line Set Successfully!",
                f"   • Length: {self.total_distance:.2f} m",
                f"   • Interval: {self.zero_interval:.1f} m",
            ]
            if self.zero_start_km is not None:
                feedback.append(f"   • Start KM: {self.zero_start_km}+000")
            if self.zero_end_km is not None:
                feedback.append(f"   • End KM: {self.zero_end_km}+{int(self.total_distance):03d}")
            for line in feedback:
                self.message_text.append(line)

        else:
            # Dialog canceled
            if auto_opened and not self.zero_line_set:
                self.zero_line.setChecked(False)
            self.message_text.append("Zero line configuration canceled.")

#                                   =============================================================
#                                             ***** Visualization app 2D & 3D Ends *****
#                                   =============================================================

#                                   ==================== Utility Functions ======================

# =======================================================================================================================================
# Define function to add sphere marker:
    """ This function are used to add sphere shape points on point cloud to visualize the plotted points. """
    def add_sphere_marker(self, point, label=None, radius = 0.07, color="Red"):
        """Add a sphere marker at the specified position with optional label"""
        sphere = vtkSphereSource()
        sphere.SetRadius(radius)
        sphere.SetCenter(point[0], point[1], point[2])

        mapper = vtkPolyDataMapper()
        mapper.SetInputConnection(sphere.GetOutputPort())

        actor = vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(self.colors.GetColor3d(color))  # Use specified color
        actor.point_index = len(self.measurement_points) - 1  # Store reference

        self.renderer.AddActor(actor)
        self.measurement_actors.append(actor)

        # Add label if provided
        if label:
            text = vtk.vtkVectorText()
            text.SetText(label)

            text_mapper = vtk.vtkPolyDataMapper()
            text_mapper.SetInputConnection(text.GetOutputPort())

            text_actor = vtk.vtkFollower()
            text_actor.SetMapper(text_mapper)
            text_actor.SetScale(0.1, 0.1, 0.1)
            text_actor.AddPosition(point[0] + 0.15, point[1] + 0.15, point[2])
            text_actor.GetProperty().SetColor(self.colors.GetColor3d("Green"))

            self.renderer.AddActor(text_actor)
            text_actor.SetCamera(self.renderer.GetActiveCamera())
            self.measurement_actors.append(text_actor)

        self.vtk_widget.GetRenderWindow().Render()
        return actor  
    
# =======================================================================================================================================
# Define function for connect two points:
    def add_line_between_points(self, p1, p2, color, label=None, show_label=True):
        line = vtkLineSource()
        line.SetPoint1(p1[0], p1[1], p1[2])
        line.SetPoint2(p2[0], p2[1], p2[2])
        
        mapper = vtkPolyDataMapper()
        mapper.SetInputConnection(line.GetOutputPort())
        
        actor = vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(self.colors.GetColor3d(color))
        actor.GetProperty().SetLineWidth(2)
        
        self.renderer.AddActor(actor)
        self.measurement_actors.append(actor)
        
        if show_label:
            # Calculate distance for label if not provided
            if label is None:
                distance_meters = sqrt(sum((p1 - p2) ** 2))
                units_suffix, conversion_factor = self.get_current_units()
                distance = distance_meters * conversion_factor
                label = f"{distance:.2f} "
            
            # Calculate midpoint for label position
            midpoint = [(p1[0] + p2[0])/2, 
                        (p1[1] + p2[1])/2, 
                        (p1[2] + p2[2])/2]
            
            # Calculate direction vector
            direction = np.array([p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]])
            direction_norm = np.linalg.norm(direction)
            
            # Only add perpendicular offset if direction is not zero
            if direction_norm > 1e-10:  # Small threshold to avoid division by zero
                direction = direction / direction_norm
                perpendicular = np.cross(direction, [0, 0, 1])  # Cross with Z-axis for perpendicular vector
                perpendicular_norm = np.linalg.norm(perpendicular)
                
                if perpendicular_norm > 1e-10:  # Check if perpendicular vector is valid
                    perpendicular = perpendicular / perpendicular_norm
                    label_pos = midpoint + perpendicular * 0.5  # Small offset
                else:
                    # Fallback if line is vertical
                    label_pos = midpoint + np.array([0, 1, 0]) * 0.5
            else:
                # Points are identical or very close
                label_pos = midpoint
            
            text = vtk.vtkVectorText()
            text.SetText(label)
            
            text_mapper = vtk.vtkPolyDataMapper() 
            text_mapper.SetInputConnection(text.GetOutputPort())
            
            text_actor = vtk.vtkFollower()
            text_actor.SetMapper(text_mapper)
            text_actor.SetScale(0.5, 0.5, 0.5)  # Increased size to 0.5
            text_actor.AddPosition(label_pos[0], label_pos[1], label_pos[2])
            text_actor.GetProperty().SetColor(self.colors.GetColor3d("Blue"))  # Blue color
            
            self.renderer.AddActor(text_actor)
            text_actor.SetCamera(self.renderer.GetActiveCamera())
            self.measurement_actors.append(text_actor)
            
            self.vtk_widget.GetRenderWindow().Render()
            return actor
        
# =======================================================================================================================================
# Define function add angle label on point cloud data point:
    def add_angle_label(self, a, b, c, label, offset=0.8):
        """Add a label showing the angle between points a-b-c at position b"""
        # Calculate position slightly away from point b
        direction = (a - b) + (c - b)
        direction = direction / np.linalg.norm(direction)
        position = b + direction * offset
        
        text = vtk.vtkVectorText()
        text.SetText(label)
        
        text_mapper = vtk.vtkPolyDataMapper()
        text_mapper.SetInputConnection(text.GetOutputPort())
        
        text_actor = vtk.vtkFollower()
        text_actor.SetMapper(text_mapper)
        text_actor.SetScale(0.1, 0.1, 0.1)
        text_actor.AddPosition(position[0], position[1], position[2])
        text_actor.GetProperty().SetColor(self.colors.GetColor3d("White"))
        
        self.renderer.AddActor(text_actor)
        text_actor.SetCamera(self.renderer.GetActiveCamera())
        self.measurement_actors.append(text_actor)
        
        self.vtk_widget.GetRenderWindow().Render()
        
# =======================================================================================================================================
# Define a fucntion for the add text label on point cloud data
    def add_text_label(self, position, text, color="Blue", scale=0.5, z_offset=0.0):
        """Add a text label at specified position"""
        try:
            pos = [position[0], position[1], position[2] + z_offset]
            # If text contains non-ASCII (like °), use BillboardTextActor3D
            if any(ord(c) > 127 for c in text):
                text_actor = vtk.vtkBillboardTextActor3D()
                text_actor.SetInput(text)
                text_actor.SetPosition(pos)
                text_actor.GetTextProperty().SetColor(self.colors.GetColor3d(color))
                text_actor.GetTextProperty().SetFontSize(int(scale * 80))
                # text_actor.GetTextProperty().BoldOn()

                self.renderer.AddActor(text_actor)
                self.measurement_actors.append(text_actor)

            else:
                # Default ASCII rendering using VectorText
                text_source = vtk.vtkVectorText()
                text_source.SetText(text)

                text_mapper = vtk.vtkPolyDataMapper()
                text_mapper.SetInputConnection(text_source.GetOutputPort())

                text_actor = vtk.vtkFollower()
                text_actor.SetMapper(text_mapper)
                text_actor.SetScale(scale, scale, scale)
                text_actor.AddPosition(position[0], position[1], position[2])
                text_actor.GetProperty().SetColor(self.colors.GetColor3d(color))
                text_actor.SetCamera(self.renderer.GetActiveCamera())

                self.renderer.AddActor(text_actor)
                self.measurement_actors.append(text_actor)

            # Render update
            self.vtk_widget.GetRenderWindow().Render()

        except Exception as e:
            print(f"Error adding text label: {e}")
     
# =======================================================================================================================================
    def changeEvent(self, event):
        super(PointCloudViewer, self).changeEvent(event)
        if event.type() == QEvent.WindowStateChange:
            state = self.windowState()
            if state == Qt.WindowMaximized:
                if self.point_cloud and self.renderer:
                    self.renderer.ResetCamera()
                    if self.vtk_widget:
                        self.vtk_widget.GetRenderWindow().Render()
            elif state == Qt.WindowMinimized:
                pass # No specific action needed for minimize

# =======================================================================================================================================
    def resizeEvent(self, event):
        super(PointCloudViewer, self).resizeEvent(event)
        if self.vtk_widget:
            self.vtk_widget.GetRenderWindow().Render()


#                                   =============================================================
#                                                  *****Measurement Starts*****
#                                   =============================================================


# =======================================================================================================================================
# LOAD POINT CLOUD
    def load_point_cloud(self):
        file_dialog = QFileDialog()
        file_path, _ = file_dialog.getOpenFileName(
            self, "Open Point Cloud File", "",
            "Point Cloud Files (*.ply *.pcd *.xyz);;All Files (*)")
        if not file_path:
            return
        try:
            # --- Store the loaded file path and name for later use ---
            self.loaded_file_path = file_path
            self.loaded_file_name = os.path.splitext(os.path.basename(file_path))[0]
            # Show progress bar with file name
            self.show_progress_bar(file_path)
            self.update_progress(10, "Starting file loading...")
            # Read the file directly without intermediate processing steps
            if file_path.endswith('.ply') or file_path.endswith('.pcd'):
                self.update_progress(30, "Loading point cloud data...")
                self.point_cloud = o3d.io.read_point_cloud(file_path)
  
            elif file_path.endswith('.xyz'):
                self.update_progress(30, "Loading XYZ data...")
                # For XYZ files, use numpy's faster loading
                data = np.loadtxt(file_path, usecols=(0, 1, 2)) # Only load XYZ columns
                self.point_cloud = o3d.geometry.PointCloud()
                self.point_cloud.points = o3d.utility.Vector3dVector(data[:, :3])
            if not self.point_cloud.has_points():
                raise ValueError("No points found in the file.")
            # Skip color processing if not needed for faster loading
            if self.point_cloud.has_colors():
                self.update_progress(70, "Processing colors...")
            else:
                self.update_progress(70, "Preparing visualization...")
            # Directly convert to VTK format without intermediate steps
            self.update_progress(90, "Creating visualization...")
            self.display_point_cloud()
            # self.start_button.setEnabled(True)
            # Final update before hiding
            self.update_progress(100, "Loading complete!")
            QTimer.singleShot(100, self.hide_progress_bar)
        except Exception as e:
            self.hide_progress_bar()

# =======================================================================================================================================
    def load_point_cloud_from_path(self, file_path: str):
        """
        Load a point cloud from a given file path without showing QFileDialog.
        Used for auto-loading point clouds linked to a project after creating a new worksheet.
        Reuses the same logic as load_point_cloud() for consistency.
        """
        if not file_path or not os.path.exists(file_path):
            self.message_text.append(f"Point cloud file not found or invalid: {file_path}")
            return False

        try:
            # Store the loaded file path and name
            self.loaded_file_path = file_path
            self.loaded_file_name = os.path.splitext(os.path.basename(file_path))[0]

            # Show progress bar with file info
            self.show_progress_bar(file_path)
            self.update_progress(10, "Starting file loading...")

            # Read the file
            if file_path.lower().endswith(('.ply', '.pcd')):
                self.update_progress(30, "Loading point cloud data...")
                self.point_cloud = o3d.io.read_point_cloud(file_path)

            elif file_path.lower().endswith('.xyz'):
                self.update_progress(30, "Loading XYZ data...")
                data = np.loadtxt(file_path, usecols=(0, 1, 2))
                self.point_cloud = o3d.geometry.PointCloud()
                self.point_cloud.points = o3d.utility.Vector3dVector(data[:, :3])

            else:
                raise ValueError(f"Unsupported file format: {os.path.splitext(file_path)[1]}")

            if not self.point_cloud.has_points():
                raise ValueError("No points found in the file.")

            if self.point_cloud.has_colors():
                self.update_progress(70, "Processing colors...")
            else:
                self.update_progress(70, "Preparing visualization...")

            self.update_progress(90, "Creating visualization...")
            self.display_point_cloud()

            self.update_progress(100, "Loading complete!")
            QTimer.singleShot(500, self.hide_progress_bar)

            self.message_text.append(f"Successfully loaded point cloud: {os.path.basename(file_path)}")
            return True

        except Exception as e:
            self.hide_progress_bar()
            self.message_text.append(f"Failed to load point cloud '{os.path.basename(file_path)}': {str(e)}")
            QMessageBox.warning(self, "Load Failed", f"Could not load point cloud:\n{file_path}\n\nError: {str(e)}")
            return False

# =======================================================================================================================================
    def display_point_cloud(self):
        if not self.point_cloud:
            return
        # Clear previous point cloud if any
        if self.point_cloud_actor:
            self.renderer.RemoveActor(self.point_cloud_actor)
        self.update_progress(92, "Converting to VTK format...")
        # Convert Open3D point cloud to VTK format
        points = np.asarray(self.point_cloud.points)
        # Create VTK points
        vtk_points = vtk.vtkPoints()
        for i, point in enumerate(points):
            vtk_points.InsertNextPoint(point[0], point[1], point[2])
            # Update progress every 1000 points
            if i % 1000 == 0:
                progress = 92 + int(6 * (i / len(points)))
                self.update_progress(min(progress, 98), f"Processing points: {i}/{len(points)}")
        # Create VTK polydata
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(vtk_points)
        # Create vertex cells
        vertices = vtk.vtkCellArray()
        for i in range(points.shape[0]):
            vertices.InsertNextCell(1)
            vertices.InsertCellPoint(i)
        polydata.SetVerts(vertices)
        # Add color information if available
        if self.point_cloud.has_colors():
            self.update_progress(95, "Processing colors...")
            colors = np.asarray(self.point_cloud.colors) * 255 # Convert from 0-1 to 0-255
            vtk_colors = vtk.vtkUnsignedCharArray()
            vtk_colors.SetNumberOfComponents(3)
            vtk_colors.SetName("Colors")
            for color in colors:
                vtk_colors.InsertNextTuple3(color[0], color[1], color[2])
            polydata.GetPointData().SetScalars(vtk_colors)
        # Create mapper and actor
        self.update_progress(97, "Creating visualization...")
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(polydata)
        self.point_cloud_actor = vtk.vtkActor()
        self.point_cloud_actor.SetMapper(mapper)
        self.point_cloud_actor.GetProperty().SetPointSize(2)
        # Only set color if no vertex colors are present
        if not self.point_cloud.has_colors():
            self.point_cloud_actor.GetProperty().SetColor(self.colors.GetColor3d("Black"))
        self.renderer.AddActor(self.point_cloud_actor)
        self.renderer.ResetCamera()
        self.update_progress(99, "Finalizing...")
        self.vtk_widget.GetRenderWindow().Render()
        self.update_progress(100, "Ready!")

# =======================================================================================================================================
# Define the function for the update the mesurement metrics as per the selected metrics::
    def update_measurement_metrics(self):
        """Update all measurements when the units change"""
        if not hasattr(self, 'measurement_points') or not self.measurement_points:
            return
        
        # Get the current units suffix
        units_suffix = self.get_units_suffix()
        
        # Update all measurement labels
        for actor in self.measurement_actors:
            if isinstance(actor, vtk.vtkFollower):
                try:
                    text_source = actor.GetMapper().GetInputConnection(0, 0).GetProducer()
                    if isinstance(text_source, vtk.vtkVectorText):
                        text = text_source.GetText()
                        if text and any(x in text for x in ['m', 'ft', 'cm', 'mm']):
                            # This is a measurement label - update it
                            if '=' in text:  # Angle label
                                continue  # Don't modify angle labels
                            
                            # Extract the numeric value
                            try:
                                value_str = text.split('=')[-1].strip().rstrip('m').rstrip('cm').rstrip('mm')
                                value_meters = float(value_str)
                                converted_value = self.convert_to_current_units(value_meters)
                                new_text = f"{converted_value:.2f}{units_suffix}"
                                text_source.SetText(new_text)
                            except:
                                continue
                except:
                    continue
        
        self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================
    def get_current_units(self):
        """Get the current units and conversion factor from meters"""
        current_units = self.metrics_combo.currentText()
        
        if current_units == "Meter":
            return "m", 1.0
        elif current_units == "Centimeter":
            return "cm", 100.0
        elif current_units == "Millimeter":
            return "mm", 1000.0
        return "m", 1.0
    
# =======================================================================================================================================
# Convert the metrics from meter to cm and mm::
    def convert_to_current_units(self, value_in_meters):
        """Convert a value in meters to the currently selected units"""
        current_units = self.metrics_combo.currentText()
        
        if current_units == "Meter":
            return value_in_meters
        elif current_units == "Centimeter":
            return value_in_meters * 100
        elif current_units == "Millimeter":
            return value_in_meters * 1000
        return value_in_meters

# =======================================================================================================================================
# Define the function for the get curent metrics of measurement
    def get_units_suffix(self):
        """Get the suffix for the current units"""
        current_units = self.metrics_combo.currentText()
        
        if current_units == "Meter":
            return "m"
        elif current_units == "Centimeter":
            return "cm"
        elif current_units == "Millimeter":
            return "mm"
        return "" 
         
# =======================================================================================================================================
    def get_line_angle(self):
        """Get angle for straight line from user input"""
        angle, ok = QInputDialog.getDouble(
            self, 
            "Line Angle", 
            "Enter angle for straight line (degrees):", 
            value=0.0, minValue=0.0, maxValue=359.9, decimals=1
        )
        
        if ok:
            return angle
        return None

# =======================================================================================================================================
# Define function for update measurement:
    def update_measurement_widget(self):
        if not self.measurement_widget:
            return
        
        # Clear all lines in the widget
        self.measurement_widget.lines = []
        
        # Redraw lines based on remaining points
        if self.current_measurement == 'polygon' and len(self.measurement_points) >= 2:
            points = self.measurement_widget.points[:len(self.measurement_points)]
            for i in range(len(points)-1):
                self.measurement_widget.add_line((points[i], points[i+1]))
            self.measurement_widget.update()

#========================================================================================================================================
# Define the funtion for the Get Measurement Data:   
    def get_current_measurement_data(self):
        """Get the current measurement data from the output list"""
        output_text = []
        for i in range(self.output_list.count()):
            output_text.append(self.output_list.item(i).text())
        
        return {
            "list_name": f"Measurement-{self.current_measurement_index + 1}",
            "measurement_type": self.current_measurement if hasattr(self, 'current_measurement') else "Unknown",
            "output_text": output_text,
            "timestamp": datetime.now().isoformat()
        }

# =======================================================================================================================================
# Define function for the connect the points which is already drawn on point cloud data:
    def connect_signals(self):
        # Measurement button connections
        self.start_button.clicked.connect(self.start_measurement)
        self.save_layer_button.clicked.connect(self.Save_layers_measurements)
         
        self.vertical_line_action.triggered.connect(lambda: self.set_measurement_type('vertical_line'))
        self.horizontal_line_action.triggered.connect(lambda: self.set_measurement_type('horizontal_line'))
        self.measurement_line_action.triggered.connect(lambda: self.set_measurement_type('measurement_line'))
        self.polygon_button.clicked.connect(lambda: self.set_measurement_type('polygon'))
        self.round_pillar_polygon_button.clicked.connect(lambda: self.set_measurement_type('round_pillar_polygon'))
        self.baseline_button.clicked.connect(self.baseline)
        self.inclination_button.clicked.connect(self.calculate_inclination_angle)
        self.pillar_dimension.clicked.connect(self.process_pillar_dimension)
        self.complete_polygon_button.clicked.connect(self.complete_polygon)
        self.complete_curve_button.clicked.connect(self.complete_curve)
        self.presized_button.clicked.connect(self.handle_presized_button)
        self.cut_hill_button.clicked.connect(self.process_cut_hill_volume)

        self.Ohe_angle_button.clicked.connect(lambda: self.set_measurement_type('ohe_pole_angle_with_rail'))
        self.defect_angle_button.clicked.connect(lambda: self.set_measurement_type('defect_angle'))
        self.all_angle_button.clicked.connect(lambda: self.set_measurement_type('all_angles'))
        # distance between catenary wire to rail level
        self.elivation_btn_catenary_raillevel_button.clicked.connect(lambda: self.set_measurement_type('distance_between_catenary_to_raillevel'))
        # distance between contact wire to rail level
        self.elivation_btn_contact_raillevel_button.clicked.connect(lambda: self.set_measurement_type('distance_between_contact_to_raillevel'))
        self.round_pillar_polygon_button.clicked.connect(lambda: self.set_measurement_type('round_pillar_polygon'))
        self.vertical_line_action.triggered.connect(lambda: self.set_measurement_type('vertical_line'))
        self.horizontal_line_action.triggered.connect(lambda: self.set_measurement_type('horizontal_line'))
        self.measurement_line_action.triggered.connect(lambda: self.set_measurement_type('measurement_line'))
        self.contact_action.triggered.connect(lambda: self.set_measurement_type('contact_wire_points'))
        self.catenary_action.triggered.connect(lambda: self.set_measurement_type('distance_between_catenary_to_contact'))

        self.crop_button.clicked.connect(self.crop_selected_area)
        self.save_crop_button.clicked.connect(self.save_cropped_data)

        self.height_check.stateChanged.connect(self.handle_height_checkbox)
        self.depth_check.stateChanged.connect(self.handle_depth_checkbox)
        self.volume_check.stateChanged.connect(self.handle_volume_checkbox)

        self.measurement_check.stateChanged.connect(self.toggle_measurement_mode)
        self.filling_check.stateChanged.connect(self.toggle_filling_mode)
        self.cutting_check.stateChanged.connect(self.toggle_cutting_mode)
        self.extraction_check.stateChanged.connect(self.toggle_extraction_mode)
        self.railway_measurement_check.toggled.connect(self.toggle_railway_measurement_mode)
        self.save_button.clicked.connect(self.save_current_design_layer)

        # Connect UI signals
        self.reset_action_button.clicked.connect(self.reset_action)
        self.reset_all_button.clicked.connect(self.reset_all)
        self.preview_button.clicked.connect(self.on_curve_button_clicked)
        self.elivation_angle_button.clicked.connect(self.on_elivation_angle_button_clicked)
        self.threed_map_button.clicked.connect(self.preview_lines_on_3d)

        # MeasurementNewDialog.
        
        # Top section button clicks handling
        self.create_project_button.clicked.connect(self.open_create_project_dialog)
        self.design_button.clicked.connect(self.handle_design_button_click)
        self.construction_button.clicked.connect(self.handle_construction_button_click)
        self.measurement_button.clicked.connect(self.handle_measurement_button_click)
        self.layers_button.clicked.connect(self.show_merger_config_dialog)
        self.load_button.clicked.connect(self.load_point_cloud)
        self.help_button.clicked.connect(self.show_help_dialog)
        # self.setting_button.clicked.connect(self.show_help_dialog)

        # new button clicks handling in topsection dropdown
        self.new_worksheet_button.clicked.connect(self.open_new_worksheet_dialog)
        self.new_design_button.clicked.connect(self.open_create_new_design_layer_dialog)
        self.new_construction_button.clicked.connect(self.open_construction_new_dialog)
        self.new_measurement_button.clicked.connect(self.open_new_measurement_dialog)

        # existing button clicks handling in topsection dropdown
        self.existing_worksheet_button.clicked.connect(self.open_existing_worksheet)
        self.existing_measurement_button.clicked.connect(self.open_existing_measurement_layers)
        
        # Connect checkbox signals
        self.zero_line.stateChanged.connect(lambda state: self.on_checkbox_changed(state, 'zero'))
        self.surface_baseline.stateChanged.connect(lambda state: self.on_checkbox_changed(state, 'surface'))
        self.construction_line.stateChanged.connect(lambda state: self.on_checkbox_changed(state, 'construction'))
        self.road_surface_line.stateChanged.connect(lambda state: self.on_checkbox_changed(state, 'road_surface'))
        self.bridge_zero_line.stateChanged.connect(lambda state: self.on_checkbox_changed(state, 'zero'))
        self.projection_line.stateChanged.connect(lambda state: self.on_checkbox_changed(state, 'projection_line'))
        self.construction_dots_line.stateChanged.connect(lambda state: self.on_checkbox_changed(state, 'construction_dots'))
        # self.material_line.stateChanged.connect(lambda state: self.on_checkbox_changed(state, 'material'))
        self.deck_line.stateChanged.connect(lambda state: self.on_checkbox_changed(state, 'deck_line'))
        
        # Connect pencil button signals
        self.zero_pencil.clicked.connect(self.edit_zero_line)
        self.bridge_zero_pencil.clicked.connect(self.edit_zero_line)
        self.construction_dots_pencil.clicked.connect(self.edit_construction_dots_line)
        self.deck_pencil.clicked.connect(self.edit_deck_line)
        self.add_material_line_button.clicked.connect(self.open_material_line_dialog)
        # self.material_pencil.clicked.connect(self.edit_material_line)
        
        # Connect slider and scrollbar signals
        self.volume_slider.valueChanged.connect(self.volume_changed)
        
        # Connect graph hover event
        self.cid_hover = self.canvas.mpl_connect('motion_notify_event', self.on_hover)
        
        # Connect undo/redo buttons
        self.undo_button.clicked.connect(self.undo_graph)
        self.redo_button.clicked.connect(self.redo_graph)
        
        # Add both left and right click events
        self.vtk_widget.GetRenderWindow().GetInteractor().AddObserver(
            "LeftButtonPressEvent", self.on_click)
        # Add key press event (Space bar for freeze/unfreeze, Escape for plotting toggle)
        self.vtk_widget.GetRenderWindow().GetInteractor().AddObserver(
            "KeyPressEvent", self.on_key_press)

# =======================================================================================================================================
    def rotate_up(self, degrees=15):
        """Rotate the view upward"""
        camera = self.renderer.GetActiveCamera()
        camera.Elevation(-degrees)  # Negative for upward rotation
        camera.OrthogonalizeViewUp()
        self.vtk_widget.GetRenderWindow().Render()

    def rotate_down(self, degrees=15):
        """Rotate the view downward"""
        camera = self.renderer.GetActiveCamera()
        camera.Elevation(degrees)  # Positive for downward rotation
        camera.OrthogonalizeViewUp()
        self.vtk_widget.GetRenderWindow().Render()

    def rotate_left(self, degrees=15):
        """Rotate the view to the left"""
        camera = self.renderer.GetActiveCamera()
        camera.Azimuth(-degrees)  # Negative for left rotation
        camera.OrthogonalizeViewUp()
        self.vtk_widget.GetRenderWindow().Render()

    def rotate_right(self, degrees=15):
        """Rotate the view to the right"""
        camera = self.renderer.GetActiveCamera()
        camera.Azimuth(degrees)  # Positive for right rotation
        camera.OrthogonalizeViewUp()
        self.vtk_widget.GetRenderWindow().Render()
            
# =======================================================================================================================================
# Define function for Key press handler for Space bar freeze/unfreeze
    def on_key_press(self, obj, event):
        """Handle key press events"""
        key = obj.GetKeySym()
        # Rotation controls
        if key == 'Up':
            self.rotate_up(15)
            return
        elif key == 'Down':
            self.rotate_down(15)
            return
        elif key == 'Right':
            self.rotate_left(15)
            return
        elif key == 'Left':
            self.rotate_right(15)
            return
        # Existing key handling
        if key == 'space': # Space bar for freeze/unfreeze
            self.freeze_view = not self.freeze_view
            interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
            if self.freeze_view:
                interactor.SetInteractorStyle(None) # Disable interaction
                # self.output_list.addItem("View frozen - press Space to unfreeze")
            else:
                # Re-enable default interaction
                style = vtkInteractorStyleTrackballCamera()
                interactor.SetInteractorStyle(style)
                # self.output_list.addItem("View unfrozen")
            self.vtk_widget.GetRenderWindow().Render()

        if key == 'escape' and self.active_line_type and self.current_points:
            self.finish_current_polyline()
            self.message_text.append(f"{self.active_line_type.replace('_', ' ').title()} completed with Escape key")

# =======================================================================================================================================
# Define function for start measurement:
    def start_measurement(self):
        self.measurement_active = True
        self.measurement_group.setEnabled(True)
        self.action_group.setEnabled(True)  # This line activates the Action section
        
        # Enable all checkboxes in Action section
        self.measurement_check.setEnabled(True)
        self.filling_check.setEnabled(True)
        self.cutting_check.setEnabled(True)
        self.extraction_check.setEnabled(True)
        self.railway_measurement_check.setEnabled(True)
        
        self.start_button.setEnabled(False)
        
        # Set the title background color to green
        self.measurement_group.setStyleSheet("""
            QGroupBox {                    
                border: 1px solid gray;
                border-radius: 5px;
                margin: 0.2em;
                margin-top: 0.5em;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px;
                background-color: #4CAF50;  /* Green color background of "start measurement" section after successfully point cloud file loaded */
                color: white;  /* White text for better contrast */
            }
        """)
        
        # Also set the Action section title to green to show it's active
        self.action_group.setStyleSheet("""
            QGroupBox {
                border: 1px solid gray;
                border-radius: 5px;
                margin-top: 0.5em;
                padding-left: 7px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px;
                background-color: #4CAF50;
                color: white;
            }
        """)
        
        # Reset any previous measurements
        self.reset_action()

# =======================================================================================================================================
    def on_click(self, obj, event):
        if not self.measurement_active or not self.current_measurement or self.freeze_view or not self.plotting_active:
            return

        interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        pos = interactor.GetEventPosition()

        # First try using cell picker which can pick between points
        cell_picker = vtk.vtkCellPicker()
        cell_picker.SetTolerance(0.0005)  # Small tolerance for accurate picking
        cell_picker.Pick(pos[0], pos[1], 0, self.renderer)
        
        clicked_point = None
        
        if cell_picker.GetCellId() != -1:
            # Get the picked position in world coordinates
            clicked_point = np.array(cell_picker.GetPickPosition())
            
            # Find the nearest actual point in the point cloud to our picked position
            points = np.asarray(self.point_cloud.points)
            if len(points) > 0:
                distances = np.sum((points - clicked_point)**2, axis=1)
                nearest_idx = np.argmin(distances)
                clicked_point = points[nearest_idx]
        
        # If still no point found, use the neighborhood search
        if clicked_point is None:
            clicked_point = self.find_nearest_point_in_neighborhood(pos)
        
        if clicked_point is None:
            return  # No point found

        if self.current_measurement == 'zero_line':
            self.zero_points.append(clicked_point)
            label = "Start" if len(self.zero_points) == 1 else "End"
            actor = self.add_sphere_marker(clicked_point, label, color="purple")
            self.temp_zero_actors.append(actor)
            if len(self.zero_points) == 2:
                self.plotting_active = False
                dialog = ZeroLineDialog(self.zero_points[0], self.zero_points[1], parent=self)
                if dialog.exec_() == QDialog.Accepted:
                    try:
                        p1, p2 = dialog.get_points()
                        if p1 is not None and p2 is not None:
                            self.zero_start_point = p1
                            self.zero_end_point = p2
                        self.zero_start_km = int(dialog.km1_edit.text() or 0)
                        self.zero_start_chain = float(dialog.chain1_edit.text() or 0)
                        self.zero_end_km = int(dialog.km2_edit.text() or 0)
                        self.zero_end_chain = float(dialog.chain2_edit.text() or 0)
                        self.zero_interval = int(dialog.interval_edit.text() or 20)
                        self.zero_physical_dist = np.linalg.norm(self.zero_end_point - self.zero_start_point)
                        self.total_distance = self.zero_physical_dist
                        self.zero_start_z = self.zero_start_point[2] # Set reference zero elevation
                        self.zero_line_set = True
                        self.zero_start_actor = self.temp_zero_actors[0]
                        self.zero_end_actor = self.temp_zero_actors[1]
                        self.zero_line_actor = self.add_line_between_points(self.zero_start_point, self.zero_end_point, "purple", show_label=False)
                        self.ax.set_xlim(0, self.total_distance)
                        self.zero_graph_line, = self.ax.plot([0, self.total_distance], [0, 0], color='purple', linewidth=3)
                        self.update_chainage_ticks()
                        
                        # SHOW THE SCALE SECTION
                        self.scale_section.setVisible(True)
                        
                        # Update scale section with chainage
                        self.update_scale_ticks()
                        
                        self.canvas.draw()
                        self.figure.tight_layout()
                        self.message_text.append("Zero line saved and graph updated.")
                        self.message_text.append(f"Scale section activated with chainage: KM {self.zero_start_km}, Interval: {self.zero_interval}m")
                        
                    except ValueError:
                        QMessageBox.warning(self, "Invalid Input", "Please enter valid numbers.")
                        self.reset_zero_drawing()
                else:
                    self.reset_zero_drawing()

        if self.current_measurement == 'round_pillar_polygon':
            if len(self.measurement_points) > 0:
                # For all points after the first one, use the same Z coordinate as the first point
                first_point_z = self.measurement_points[0][2]
                clicked_point[2] = first_point_z  # Lock Z coordinate
                
            # Add point to measurement
            self.measurement_points.append(clicked_point)
            point_label = chr(65 + len(self.measurement_points) - 1)  # A, B, C, etc.
            self.add_sphere_marker(clicked_point, point_label)
            
            # Add point to measurement widget
            self.measurement_widget.add_point(QPoint(
                int(clicked_point[0] * 10 + 100),
                int(clicked_point[1] * 10 + 100)
            ))
            
            # If this is at least the second point, draw a line
            if len(self.measurement_points) >= 2:
                p1 = self.measurement_points[-2]
                p2 = self.measurement_points[-1]
                if self.current_measurement == 'round_pillar_polygon':
                    self.add_line_between_points(p1, p2, "Purple", show_label=False)
                
                # Update measurement widget
                if len(self.measurement_widget.points) >= 2:
                    points = self.measurement_widget.points
                    self.measurement_widget.add_line((points[-2], points[-1]))
                    self.measurement_widget.update()
            
            self.vtk_widget.GetRenderWindow().Render()
            return
        
        if self.current_measurement == 'ohe_pole_angle_with_rail':
            # For angle measurement, we allow up to 3 points
            self.measurement_points.append(clicked_point)

            point_label = chr(65 + len(self.measurement_points) - 1)  # A, B, C
            self.add_sphere_marker(clicked_point, point_label, radius=0.1)

            # Add point to measurement widget
            self.measurement_widget.add_point(QPoint(
                int(clicked_point[0] * 10 + 100),
                int(clicked_point[1] * 10 + 100)
            ))

            self.process_ohe_pole_angle_measurement()
            self.vtk_widget.GetRenderWindow().Render()
            return            
        
        if self.current_measurement == 'all_angles':
            self.measurement_points.append(clicked_point)

            point_label = chr(65 + len(self.measurement_points) - 1)  # A, B, C
            self.add_sphere_marker(clicked_point, point_label, radius=0.07)

            # Add point to measurement widget
            self.measurement_widget.add_point(QPoint(
                int(clicked_point[0] * 10 + 100),
                int(clicked_point[1] * 10 + 100)
            ))

            if len(self.measurement_points) >= 2:
                p1 = self.measurement_points[-2]
                p2 = self.measurement_points[-1]
                if self.current_measurement == 'all_angles':
                    self.add_line_between_points(p1, p2, "Red", show_label=False)
                
                # Update measurement widget
                if len(self.measurement_widget.points) >= 2:
                    points = self.measurement_widget.points
                    self.measurement_widget.add_line((points[-2], points[-1]))
                    self.measurement_widget.update()

            self.process_all_angles()
            self.vtk_widget.GetRenderWindow().Render()
            return

        if self.current_measurement == 'defect_angle':
            # For defect angle, we allow up to 2 points
            self.measurement_points.append(clicked_point)

            # point_label = chr(65 + len(self.measurement_points) - 1)  # A, B
            self.add_sphere_marker(clicked_point, radius=0.07)  

            # Add point to measurement widget
            self.measurement_widget.add_point(QPoint(
                int(clicked_point[0] * 10 + 100),
                int(clicked_point[1] * 10 + 100)
            ))
            self.process_defect_angle()
            self.vtk_widget.GetRenderWindow().Render()
            return

        if self.current_measurement == 'distance_between_catenary_to_raillevel':           
            # Just keep adding points, no restriction
            self.measurement_points.append(clicked_point)

            point_label = chr(65 + len(self.measurement_points) - 1)  # A, B, C...
            self.add_sphere_marker(clicked_point, point_label, radius=0.1)

            # Add point to measurement widget
            self.measurement_widget.add_point(QPoint(
                int(clicked_point[0] * 10 + 100),
                int(clicked_point[1] * 10 + 100)
            ))
            self.vtk_widget.GetRenderWindow().Render()
            return
        
        if self.current_measurement == 'distance_between_contact_to_raillevel':           
            # Just keep adding points, no restriction
            self.measurement_points.append(clicked_point)

            point_label = chr(65 + len(self.measurement_points) - 1)  # A, B, C...
            self.add_sphere_marker(clicked_point, point_label, radius=0.1)

            # Add point to measurement widget
            self.measurement_widget.add_point(QPoint(
                int(clicked_point[0] * 10 + 100),
                int(clicked_point[1] * 10 + 100)
            ))
            self.vtk_widget.GetRenderWindow().Render()
            return
        
        if self.current_measurement == 'contact_wire_points':
            self.contact_wire_points.append(clicked_point)

            point_label = chr(67 + len(self.contact_wire_points) - 1)  # C, D, E...
            self.add_sphere_marker(clicked_point, point_label, radius=0.1)

            # Add point to measurement widget
            self.measurement_widget.add_point(QPoint(
                int(clicked_point[0] * 10 + 100),
                int(clicked_point[1] * 10 + 100)
            ))
            self.vtk_widget.GetRenderWindow().Render()
            return
        
        if self.current_measurement == 'distance_between_catenary_to_contact':
            # Just keep adding points, no restriction
            self.measurement_points.append(clicked_point)

            point_label = chr(65 + len(self.measurement_points) - 1)  # A, B, C...
            self.add_sphere_marker(clicked_point, point_label, radius=0.1)

            # Add point to measurement widget
            self.measurement_widget.add_point(QPoint(
                int(clicked_point[0] * 10 + 100),
                int(clicked_point[1] * 10 + 100)
            ))
            self.vtk_widget.GetRenderWindow().Render()
            return

        # Handle measurement line case (3 base points)
        if self.current_measurement == 'measurement_line':
            # Add point to measurement_line_points
            self.measurement_line_points.append(clicked_point)
            
            # For points after first one, lock Z coordinate to first point's Z
            if len(self.measurement_line_points) > 1:
                clicked_point[2] = self.measurement_line_points[0][2]
                self.measurement_line_points[-1] = clicked_point
            
            point_label = chr(77 + len(self.measurement_line_points) - 1)  # M, N, O
            self.add_sphere_marker(clicked_point, point_label)
            
            # If we have at least 2 points, draw lines between them
            if len(self.measurement_line_points) >= 2:
                p1 = self.measurement_line_points[-2]
                p2 = self.measurement_line_points[-1]
                self.add_line_between_points(p1, p2, "Red")
            
            # When we have all 3 points, process the measurement
            if len(self.measurement_line_points) == 3:
                self.process_measurement_line()
            
            self.vtk_widget.GetRenderWindow().Render()
            return
        
        # Handle vertical line case (after measurement line is set)
        elif self.current_measurement == 'vertical_line' and hasattr(self, 'measurement_line_points'):
            # Add point to measurement_points (for vertical line)
            self.measurement_points.append(clicked_point)
            point_label = chr(65 + len(self.measurement_points) - 1)  # A, B
            
            # Visualize the point
            self.add_sphere_marker(clicked_point, point_label, color="Blue")
            
            # If we have 2 points, process the measurement
            if len(self.measurement_points) == 2:
                # Calculate and display all measurements
                self.process_measurement_with_vertical()
            
            self.vtk_widget.GetRenderWindow().Render()
            return
        
        # Handle vertical line measurement with baseline snapping
        if self.current_measurement == 'vertical_line':
            # Add point to measurement
            self.measurement_points.append(clicked_point)

            # Visualize point
            self.add_sphere_marker(clicked_point, color="Blue")                                                   
            
            # If we have two points, process the measurement
            if len(self.measurement_points) == 2:
                # For second point, snap to baseline if available
                if hasattr(self, 'baseline_actors') and self.baseline_actors:
                    baseline_point = self.find_intersection_with_baseline(self.measurement_points[0])
                    if baseline_point is not None:
                        # Replace second point with baseline point
                        self.measurement_points[1] = baseline_point
                        # Update the marker
                        for actor in reversed(self.measurement_actors):
                            if isinstance(actor.GetMapper().GetInput(), vtk.vtkSphereSource):
                                actor.SetPosition(baseline_point[0], baseline_point[1], baseline_point[2])
                                break
                        # Update the label
                        for actor in reversed(self.measurement_actors):
                            if isinstance(actor, vtk.vtkFollower):
                                try:
                                    text_source = actor.GetMapper().GetInputConnection(0, 0).GetProducer()
                                    if isinstance(text_source, vtk.vtkVectorText) and text_source.GetText() == "B":
                                        actor.SetPosition(baseline_point[0] + 0.15, 
                                                        baseline_point[1] + 0.15, 
                                                        baseline_point[2])
                                        break
                                except:
                                    continue
                
                self.process_vertical_line_measurement()
            
            self.vtk_widget.GetRenderWindow().Render()
            return
        
        # Handle horizontal line measurement
        if self.current_measurement == 'horizontal_line':
            # Add point to measurement
            self.measurement_points.append(clicked_point)

            # Visualize point
            self.add_sphere_marker(clicked_point, color="Blue") 
            
            self.process_horizontal_line_measurement()
            
            self.vtk_widget.GetRenderWindow().Render()
            return
        
        # Handle polygon measurement
        # In the on_click method, modify the polygon handling section:
        if self.current_measurement == 'polygon' :
            # For first point, just add it
            if len(self.measurement_points) == 0:
                self.measurement_points.append(clicked_point)
                point_label = 'A'  # First point is always A
                self.add_sphere_marker(clicked_point, point_label)
                
                # Add point to measurement widget
                self.measurement_widget.add_point(QPoint(
                    int(clicked_point[0] * 10 + 100),
                    int(clicked_point[1] * 10 + 100)
                ))
                self.vtk_widget.GetRenderWindow().Render()
                return
                    
            # For regular clicks (adding points)
            point_label = chr(65 + len(self.measurement_points))  # A, B, C, etc.
            self.measurement_points.append(clicked_point)
            self.add_sphere_marker(clicked_point, point_label)
            
            # Add point to measurement widget
            self.measurement_widget.add_point(QPoint(
                int(clicked_point[0] * 10 + 100),
                int(clicked_point[1] * 10 + 100)
            ))
                
            # If this is at least the second point, draw a line
            if len(self.measurement_points) >= 2:
                p1 = self.measurement_points[-2]
                p2 = self.measurement_points[-1]
                self.add_line_between_points(p1, p2, "Purple")
                
                # Update measurement widget
                if len(self.measurement_widget.points) >= 2:
                    points = self.measurement_widget.points
                    self.measurement_widget.add_line((points[-2], points[-1]))
                    self.measurement_widget.update()              

        self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================
    def find_nearest_point_in_neighborhood(self, click_pos, search_radius=4):
        """Find the nearest point in a neighborhood around the click position.
    
        Args:
            click_pos: (x, y) tuple of screen coordinates
            search_radius: radius in pixels to search around the click position
        
        Returns:
            The closest point's 3D coordinates, or None if no points found
        """
        if not hasattr(self, 'point_cloud') or not self.point_cloud:
            return None
        
        points = np.asarray(self.point_cloud.points)
        if len(points) == 0:
            return None
    
        # Convert all points to display coordinates
        renderer = self.renderer
        display_coords = []
    
        for point in points:
            try:
                display_coord = renderer.WorldToDisplay(point[0], point[1], point[2])
                display_coords.append(display_coord[:2])  # Only need x,y
            except:
                continue
    
        if not display_coords:
            return None
        
        display_coords = np.array(display_coords)
    
        # Calculate distances from click position
        distances = np.sqrt(
            (display_coords[:, 0] - click_pos[0])**2 + 
            (display_coords[:, 1] - click_pos[1])**2
        )
    
        # Find the closest point within radius
        valid_indices = np.where(distances <= search_radius)[0]
        if len(valid_indices) == 0:
            return None
    
        closest_idx = valid_indices[np.argmin(distances[valid_indices])]
    
        return points[closest_idx]
      
# =======================================================================================================================================
# Define function for the mesurement type:
    def set_measurement_type(self, m_type):
        """Set the measurement type and configure the UI and state accordingly."""
        if not self.measurement_active:
            return

        self.current_measurement = m_type
        self.measurement_started = False
        self.plotting_active = True

        # Only clear measurement points if not continuing measurement line
        if m_type != 'vertical_line' or not hasattr(self, 'measurement_line_points'):
            self.measurement_points = []

        if m_type == 'zero_line':
            self.depth_check.setChecked(False)
            self.measurement_started = True

        # Configure based on measurement type
        if m_type == 'polygon':
            self.depth_check.setChecked(False)
            self.measurement_widget.set_measurement_type('polygon')
            # Show the Complete Polygon button
            self.complete_polygon_button.setVisible(True)
            self.complete_polygon_button.setStyleSheet("""
            QPushButton {
                background-color: #98FB98;  /* Light green */
                color: black;
                border: 1px solid gray;
                padding: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #7DCE7D;  /* Slightly darker green on hover */
            }
            """)
            self.measurement_started = True
            # self.output_list.addItem("Polygon measurement started. Click points to create polygon, then click 'Complete Polygon'")

        elif m_type == 'round_pillar_polygon':
            self.depth_check.setChecked(False)
            self.measurement_widget.set_measurement_type('round_pillar_polygon')
            # Show the Complete Polygon button
            self.complete_polygon_button.setVisible(True)
            self.complete_polygon_button.setStyleSheet("""
            QPushButton {
                background-color: #98FB98;  /* Light green */
                color: black;
                border: 1px solid gray;
                padding: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #7DCE7D;  /* Slightly darker green on hover */
            }
            """)
            self.measurement_started = True
            # self.output_list.addItem("Round Pillar Polygon measurement started. Click points to create polygon, then click 'Complete Polygon'")
        else:
            # Hide the Complete Polygon button for other measurement types
            self.complete_polygon_button.setVisible(False)
            self.complete_polygon_button.setStyleSheet("")  # Reset to default style

        if m_type == 'ohe_pole_angle_with_rail':
            if not hasattr(self, 'horizontal_points') or len(self.horizontal_points) < 2:
                QMessageBox.information(self, "Info", "Please measure a horizontal distance between rails first.")
                self.plotting_active = False
                return
            
            self.plotting_active = True
            self.depth_check.setChecked(False)
            self.measurement_widget.set_measurement_type('ohe_pole_angle_with_rail')
            self.measurement_started = True

        elif m_type == 'all_angles':
            self.depth_check.setChecked(False)
            self.measurement_widget.set_measurement_type('all_angles')
            self.measurement_started = True

        if m_type == 'defect_angle':
            self.depth_check.setChecked(False)
            self.measurement_widget.set_measurement_type('defect_angle')
            self.measurement_started = True

        if m_type == 'contact_wire_points':
            self.contact_wire_points = []
            self.depth_check.setChecked(False)
            self.measurement_widget.set_measurement_type('contact_wire_points')
            self.measurement_started = True

        if m_type == 'distance_between_catenary_to_raillevel':
            if not hasattr(self, 'horizontal_points') or len(self.horizontal_points) < 2:
                QMessageBox.information(self, "Info", "Please measure a horizontal distance between rails first.")  
                self.plotting_active = False 
                return
            
            self.plotting_active = True
            self.depth_check.setChecked(False)
            self.measurement_widget.set_measurement_type('distance_between_catenary_to_raillevel')
            # Show the Complete curve button
            self.complete_curve_button.setVisible(True)
            self.complete_curve_button.setStyleSheet("""
            QPushButton {
                background-color: #98FB98;  /* Light green */
                color: black;
                border: 1px solid gray;
                padding: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #7DCE7D;  /* Slightly darker green on hover */
            }
            """)    
            self.measurement_started = True

        elif m_type == 'distance_between_contact_to_raillevel':
            if not hasattr(self, 'horizontal_points') or len(self.horizontal_points) < 2:
                QMessageBox.information(self, "Info", "Please measure a horizontal distance between rails first.")  
                self.plotting_active = False 
                return
            
            self.plotting_active = True
            self.depth_check.setChecked(False)
            self.measurement_widget.set_measurement_type('distance_between_contact_to_raillevel')
            # Show the Complete curve button
            self.complete_curve_button.setVisible(True)
            self.complete_curve_button.setStyleSheet("""
            QPushButton {
                background-color: #98FB98;  /* Light green */
                color: black;
                border: 1px solid gray;
                padding: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #7DCE7D;  /* Slightly darker green on hover */
            }
            """)    
            self.measurement_started = True

        elif m_type == 'distance_between_catenary_to_contact':
            if hasattr(self, 'contact_wire_points') and len(self.contact_wire_points) < 3:
                QMessageBox.information(self, "Info", "Please pick three or more points on the contact wire first.")   
                self.plotting_active = False
                return
            
            self.plotting_active = True
            self.depth_check.setChecked(False)
            self.measurement_widget.set_measurement_type('distance_between_catenary_to_contact')  
            # Show the Complete curve button
            self.complete_curve_button.setVisible(True)
            self.complete_curve_button.setStyleSheet("""
            QPushButton {
                background-color: #98FB98;  /* Light green */
                color: black;
                border: 1px solid gray;
                padding: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #7DCE7D;  /* Slightly darker green on hover */                                    
            }
            """)
            self.measurement_started = True

        else:
            # Hide the Complete Polygon button for other measurement types
            self.complete_curve_button.setVisible(False)
            self.complete_curve_button.setStyleSheet("")  # Reset to default style

        if m_type == 'measurement_line':
            if not hasattr(self, 'measurement_line_points'):
                self.measurement_points = []
                self.measurement_line_points = []
          
            self.depth_check.setChecked(False)
            self.volume_check.setChecked(True)
            self.measurement_widget.set_measurement_type('line')

        elif m_type == 'vertical_line':
            if (hasattr(self, 'measurement_widget') and 
                hasattr(self.measurement_widget, 'measurement_data') and
                'area_meters' in self.measurement_widget.measurement_data):
                #self.output_list.addItem("")
                self.measurement_points = []  # Clear previous points
            # elif hasattr(self, 'measurement_line_points') and len(self.measurement_line_points) >= 3:
            #     self.output_list.addItem("")
            # else:
            #     self.output_list.addItem("")
                
            self.depth_check.setChecked(False)
            self.presized_button.setVisible(True)
            self.measurement_widget.set_measurement_type('line')
            self.measurement_started = True

        elif m_type == 'horizontal_line':
            self.depth_check.setChecked(False)
            self.measurement_widget.set_measurement_type('line')
            # self.output_list.addItem("")
            self.presized_button.setVisible(True)   

            # Configure based on measurement type
        elif m_type == 'measurement_line':
            self.measurement_points = []
            self.measurement_line_points = []
            self.measurement_widget.set_measurement_type('line')
            self.output_list.addItem("Measurement line selected. Click 3 points (M, N, O) for base rectangle.")
            self.measurement_started = True
        
        elif m_type == 'vertical_line':
            if (hasattr(self, 'measurement_widget') and 
                hasattr(self.measurement_widget, 'measurement_data') and
                'area_meters' in self.measurement_widget.measurement_data):
                self.output_list.addItem("Vertical line selected. Click 2 points (A, B) for height measurement.")
                self.measurement_points = []  # Clear previous points
                self.presized_button.setVisible(True)   
            # else:
            #     self.output_list.addItem("Please complete measurement line first")
                return
        else:
            # Hide the presized button for other measurement types
            self.presized_button.setVisible(False)

# =======================================================================================================================================
    def show_measurement_controls(self, controls_to_show):
        """Show specific measurement controls based on mode"""
        # First hide all controls
        for widget in self.measurement_buttons_layout.parent().findChildren(QWidget):
            widget.setVisible(False)
        
        # Show requested controls
        control_map = {
            'line': self.line_button,
            'polygon': self.polygon_button,
            'round_pillar_polygon': self.round_pillar_polygon_button,
            'baseline': self.baseline_button,
            'inclination': self.inclination_button,
            'pillar_dimension': self.pillar_dimension,
            'depth': self.depth_check,
            'height': self.height_check,
            'volume': self.volume_check,
            'crop': self.crop_button,
            'save_crop': self.save_crop_button,
            'ohe_pole_angle': self.Ohe_angle_button,
            'all_angle': self.all_angle_button,
            'elivation_catenary_rail': self.elivation_btn_catenary_raillevel_button,
            'elivation_contact_rail': self.elivation_btn_contact_raillevel_button,
            'elivation_catenary_contact': self.elivation_btn_catenary_contact_button,
            'defect_angle': self.defect_angle_button,
            'cut_hill': self.cut_hill_button,
        }
        
        for control in controls_to_show:
            if control in control_map:
                control_map[control].setVisible(True)
        
        self.measurement_buttons_container.setVisible(True)
        self.measurement_group.setEnabled(True)

# =======================================================================================================================================
    def hide_measurement_controls(self):
        """Hide all measurement controls"""
        self.measurement_buttons_container.setVisible(False)

# =======================================================================================================================================
    def toggle_measurement_mode(self, state):
        """Handle measurement checkbox state change"""
        if state:
            # Uncheck other checkboxes
            self.filling_check.setChecked(False)
            self.cutting_check.setChecked(False)
            self.extraction_check.setChecked(False)
            self.railway_measurement_check.setChecked(False)
            
            #self.output_list.addItem("Measurement mode activated")
            # Show the measurement buttons
            self.show_measurement_controls(['line', 'polygon', 'round_pillar_polygon', 'presized', 'baseline', 'inclination', 'pillar_dimension', 'cut_hill'])
        else:
            # Hide the measurement buttons when unchecked
            self.hide_measurement_controls()
            #self.output_list.addItem("Measurement mode deactivated")

# =======================================================================================================================================
    def toggle_filling_mode(self, state):
        """Handle filling checkbox state change"""
        if state:
            # Uncheck other checkboxes
            self.measurement_check.setChecked(False)
            self.cutting_check.setChecked(False)
            self.extraction_check.setChecked(False)
            self.railway_measurement_check.setChecked(False)
            
            # self.output_list.addItem("Filling mode activated")
            # Show specific controls for filling
            self.show_measurement_controls(['polygon', 'depth', 'height', 'volume'])
        else:
            self.hide_measurement_controls()
            # self.output_list.addItem("Filling mode deactivated")

# =======================================================================================================================================
    def toggle_cutting_mode(self, state):
        """Handle cutting checkbox state change"""
        if state:
            # Uncheck other checkboxes
            self.measurement_check.setChecked(False)
            self.filling_check.setChecked(False)
            self.extraction_check.setChecked(False)
            self.railway_measurement_check.setChecked(False)
            
            # self.output_list.addItem("Cutting mode activated")
            # Show specific controls for cutting
            self.show_measurement_controls(['polygon', 'crop', 'save_crop'])
        else:
            self.hide_measurement_controls()
            # self.output_list.addItem("Cutting mode deactivated")

# =======================================================================================================================================
    def toggle_extraction_mode(self, state):
        """Handle extraction checkbox state change"""
        if state:
            # Uncheck other checkboxes
            self.measurement_check.setChecked(False)
            self.filling_check.setChecked(False)
            self.cutting_check.setChecked(False)
            self.railway_measurement_check.setChecked(False)
            
            # self.output_list.addItem("Extraction mode activated")
            # Show only polygon button in measurement section
            self.show_measurement_controls(['polygon'])
            
            # Show surface-related buttons if we have a completed polygon
            if hasattr(self, 'measurement_points') and len(self.measurement_points) >= 3:
                self.show_surface_controls_in_measurement()
        else:
            self.hide_measurement_controls()
            # Hide all digging point sections
            self.digging_point_input.setVisible(False)
            self.connection_group.setVisible(False)
            self.polygon_digging_group.setVisible(False)
            # self.output_list.addItem("Extraction mode deactivated")
            
            # Hide surface controls
            self.hide_surface_controls()

# =======================================================================================================================================
    def toggle_railway_measurement_mode(self, state):
        """Handle railway measurement checkbox state change""" 
        if state:
            self.measurement_check.setChecked(False)
            self.filling_check.setChecked(False)
            self.cutting_check.setChecked(False)
            self.extraction_check.setChecked(False)

            self.show_measurement_controls(['line', 'ohe_pole_angle', 'all_angle', 'defect_angle', 'elivation_catenary_rail', 'elivation_contact_rail', 'elivation_catenary_contact', 'intersection_angle']) 
        else:
            self.hide_measurement_controls()

# =======================================================================================================================================       
# Define the function for Height Checkbox & Input Box:
    def handle_height_checkbox(self, state):
        """Handle height checkbox state change for polygon measurement"""
        if state == Qt.Checked:
            units_suffix = self.get_units_suffix()
            self.height_input.setPlaceholderText(f"Enter height ({units_suffix})")
            self.height_input.setVisible(True)
            self.height_input.setFocus()
            
            # Uncheck depth if height is checked
            self.depth_check.setChecked(False)
            self.depth_input.setVisible(False)
            
            # If we have a completed polygon, prompt user to enter height
            if (self.current_measurement == 'polygon' and 
                hasattr(self, 'measurement_widget') and 
                'area' in self.measurement_widget.measurement_data):
                # self.output_list.addItem("")
                pass
        else:
            self.height_input.setVisible(False)
            # Remove any height visualization if unchecked
            self.remove_height_visualization()

# =======================================================================================================================================
# Define the function for depth Checkbox & Input Box:
    def handle_depth_checkbox(self, state):
        """Handle depth checkbox state change for polygon measurement"""
        if state == Qt.Checked:
            units_suffix = self.get_units_suffix()
            self.depth_input.setPlaceholderText(f"Enter depth ({units_suffix})")
            self.depth_input.setVisible(True)
            self.depth_input.setFocus()
            
            # Uncheck height if depth is checked
            self.height_check.setChecked(False)
            self.height_input.setVisible(False)
            
            # If we have a completed polygon, prompt user to enter depth
            if (self.current_measurement == 'polygon' and 
                hasattr(self, 'measurement_widget') and 
                'area' in self.measurement_widget.measurement_data):
                # self.output_list.addItem("")
                pass
        else:
            self.depth_input.setVisible(False)
            # Remove any depth visualization if unchecked
            self.remove_depth_visualization()

# =======================================================================================================================================
    def handle_volume_checkbox(self, state):
        """Handle volume checkbox state change for polygon measurement"""
        if state == Qt.Checked:
            if not hasattr(self.measurement_widget, 'measurement_data') or 'area' not in self.measurement_widget.measurement_data:
                #self.output_list.addItem("Please complete polygon measurement first")
                self.volume_check.setChecked(False)
                return
                
            # Check if we have height or depth value
            if self.height_check.isChecked() and self.height_input.text():
                try:
                    height_value = float(self.height_input.text())
                    if height_value <= 0:
                        #self.output_list.addItem("Height must be greater than 0")
                        self.volume_check.setChecked(False)
                        return
                        
                    self.calculate_and_visualize_polygon_volume(height_value, is_height=True)
                    
                except ValueError:
                    self.output_list.addItem("Please enter a valid height value")
                    self.volume_check.setChecked(False)
                    
            elif self.depth_check.isChecked() and self.depth_input.text():
                try:
                    depth_value = float(self.depth_input.text())
                    if depth_value <= 0:
                        # self.output_list.addItem("Depth must be greater than 0")
                        self.volume_check.setChecked(False)
                        return
                        
                    self.calculate_and_visualize_polygon_volume(depth_value, is_height=False)
                    
                except ValueError:
                    self.output_list.addItem("Please enter a valid depth value")
                    self.volume_check.setChecked(False)
            else:
                #self.output_list.addItem("Please enter height or depth value first")
                self.volume_check.setChecked(False)
        else:
            # Remove volume visualization when unchecked
            self.remove_volume_visualization()

# =======================================================================================================================================
    def calculate_and_visualize_polygon_volume(self, value, is_height=True):
        """Calculate and visualize polygon volume with given height or depth"""
        if len(self.measurement_points) < 3:
            return
            
        # Get current units and conversion factors
        units_suffix, conversion_factor = self.get_current_units()
        area_suffix = "square " + units_suffix.replace("m", "") if units_suffix != "m" else "square meter"
        volume_suffix = "cubic " + units_suffix.replace("m", "") if units_suffix != "m" else "cubic meter"
        
        # Convert value to meters first if needed
        if units_suffix == "cm":
            value_meters = value / 100
        elif units_suffix == "mm":
            value_meters = value / 1000
        else:
            value_meters = value
            
        area_meters = self.measurement_widget.measurement_data['area'] / (conversion_factor ** 2)
        volume_meters = area_meters * value_meters
        
        # Convert volume to current unit
        if units_suffix == "cm":
            volume = volume_meters * 1000000  # m³ to cm³
        elif units_suffix == "mm":
            volume = volume_meters * 1000000000  # m³ to mm³
        else:
            volume = volume_meters  # m³
        
        # Visualize the 3D polygon prism
        self.visualize_polygon_prism(value_meters, is_height)
        
        # Output results
        if is_height:
            self.output_list.addItem(f"Polygon volume (using height):")
            self.output_list.addItem(f"Area: {self.measurement_widget.measurement_data['area']:.3f} {area_suffix}")
            self.output_list.addItem(f"Height: {value:.3f} {units_suffix}")
        else:
            self.output_list.addItem(f"Polygon volume (using depth):")
            self.output_list.addItem(f"Area: {self.measurement_widget.measurement_data['area']:.3f} {area_suffix}")
            self.output_list.addItem(f"Depth: {value:.3f} {units_suffix}")
            
        self.output_list.addItem(f"Volume: {volume:.3f} {volume_suffix}")

# =======================================================================================================================================
    def visualize_polygon_prism(self, value_meters, is_height=True):
        """Visualize polygon prism with given height or depth"""
        if len(self.measurement_points) < 3:
            return
            
        # Remove any existing volume visualization
        self.remove_volume_visualization()
        
        # Create bottom face (original polygon)
        bottom_face = self.measurement_points
        
        # Create top face (extruded upward for height, downward for depth)
        if is_height:
            top_face = [p + np.array([0, 0, value_meters]) for p in bottom_face]
        else:
            top_face = [p - np.array([0, 0, value_meters]) for p in bottom_face]
        
        # Create all faces of the prism (sides + top and bottom)
        faces = [bottom_face]  # Bottom face
        
        # Create side faces
        n = len(bottom_face)
        for i in range(n):
            next_i = (i + 1) % n
            side_face = [
                bottom_face[i],
                bottom_face[next_i],
                top_face[next_i],
                top_face[i]
            ]
            faces.append(side_face)
        
        faces.append(top_face)  # Top face
        
        # Create VTK objects for each face
        for i, face_points in enumerate(faces):
            # Create polygon from points
            polygon = vtk.vtkPolygon()
            polygon.GetPointIds().SetNumberOfIds(len(face_points))
            for j, point in enumerate(face_points):
                polygon.GetPointIds().SetId(j, j)
            
            polygons = vtk.vtkCellArray()
            polygons.InsertNextCell(polygon)
            
            polygonPolyData = vtk.vtkPolyData()
            polygonPolyData.SetPoints(vtk.vtkPoints())
            for point in face_points:
                polygonPolyData.GetPoints().InsertNextPoint(point)
            polygonPolyData.SetPolys(polygons)
            
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(polygonPolyData)
            
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            
            # Use different colors for bottom/top and sides
            if i == 0:  # Bottom face
                actor.GetProperty().SetColor(self.colors.GetColor3d("Blue"))
            elif i == len(faces) - 1:  # Top face
                actor.GetProperty().SetColor(self.colors.GetColor3d("Blue"))
            else:       # Side faces
                actor.GetProperty().SetColor(self.colors.GetColor3d("LightBlue"))
            
            actor.GetProperty().SetOpacity(0.5)  # Semi-transparent
            actor.GetProperty().SetEdgeVisibility(1)
            actor.GetProperty().SetEdgeColor(self.colors.GetColor3d("White"))
            actor.is_volume_visualization = True  # Mark as volume visualization
            
            self.renderer.AddActor(actor)
            self.measurement_actors.append(actor)
        
        # Add height/depth label
        centroid = np.mean(bottom_face, axis=0)
        if is_height:
            label_pos = centroid + np.array([0, 0, value_meters/2])
            self.add_text_label(label_pos, f"h={value_meters:.2f}m", "White")
        else:
            label_pos = centroid - np.array([0, 0, value_meters/2])
            self.add_text_label(label_pos, f"d={value_meters:.2f}m", "White")
        
        self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================
    def remove_height_visualization(self):
        """Remove height visualization actors"""
        actors_to_remove = []
        for actor in self.measurement_actors:
            if hasattr(actor, 'is_height_visualization'):
                actors_to_remove.append(actor)
        
        for actor in actors_to_remove:
            self.renderer.RemoveActor(actor)
            if actor in self.measurement_actors:
                self.measurement_actors.remove(actor)
        
        self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================
    def remove_depth_visualization(self):
        """Remove depth visualization actors"""
        actors_to_remove = []
        for actor in self.measurement_actors:
            if hasattr(actor, 'is_depth_visualization'):
                actors_to_remove.append(actor)
        
        for actor in actors_to_remove:
            self.renderer.RemoveActor(actor)
            if actor in self.measurement_actors:
                self.measurement_actors.remove(actor)
        
        self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================    
    def remove_volume_visualization(self):
        """Remove all volume visualization actors"""
        actors_to_remove = []
        
        # Find all volume visualization actors (transparent actors)
        for actor in self.measurement_actors:
            if isinstance(actor.GetMapper().GetInput(), vtk.vtkPolyData):
                if actor.GetProperty().GetOpacity() < 1.0:  # Our volume visualizations are transparent
                    actors_to_remove.append(actor)
        
        # Also remove any height/depth labels
        for actor in self.measurement_actors:
            if isinstance(actor, vtk.vtkFollower):
                try:
                    text_source = actor.GetMapper().GetInputConnection(0, 0).GetProducer()
                    if isinstance(text_source, vtk.vtkVectorText):
                        text = text_source.GetText()
                        if text and (text.startswith("h=") or text.startswith("d=")):
                            actors_to_remove.append(actor)
                except:
                    continue
        
        # Remove the actors
        for actor in actors_to_remove:
            self.renderer.RemoveActor(actor)
            if actor in self.measurement_actors:
                self.measurement_actors.remove(actor)
        
        self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================
    def hide_surface_controls(self):
        """Hide all surface-related controls in the Measurement section"""
        if hasattr(self, 'surface_controls_container'):
            self.surface_controls_container.setVisible(False)
        if hasattr(self, 'surface_selection_group'):
            self.surface_selection_group.setVisible(False)

# =======================================================================================================================================
# Define function for Angle Measurement between the Ohe Pole and Rail level:
    def process_ohe_pole_angle_measurement(self):
        if len(self.measurement_points) != 3:
            # self.output_list.addItem("Need exactly 3 points to compute angles.")
            return
        self.plotting_active = False

        if not hasattr(self, "horizontal_points") or len(self.horizontal_points) < 2:
            return

        # --- Extract picked points ---
        left_point = self.horizontal_points[0]
        right_point = self.horizontal_points[1]

        self.left_point = left_point
        self.right_point = right_point

        x1, y1, z1 = left_point
        x2, y2, z2 = right_point

        # --- Step 3: Mean Top of Rail (MTOR) ---
        z_mean = (z1 + z2) / 2.0
        baseline_z = z_mean
        self.OHE_baseline_z = baseline_z  # Store for later use

        self.output_list.addItem(f"Rail heights: Left={z1:.2f}, Right={z2:.2f}, MTOR={z_mean:.2f}")

        # --- Step 1: Extract points ---
        # Top of OHE pole (picked)
        top_pt = np.array(self.measurement_points[0])

        # Intersection point (directly below top on baseline)
        inter_pt = np.array(self.measurement_points[1])
        inter_pt[2] = self.OHE_baseline_z # snap to baseline

        # Track point (same Y,Z as intersection, user’s X)
        raw_track_pt = np.array(self.measurement_points[2])
        track_pt = np.array([ 
            inter_pt[0],                  
            raw_track_pt[1],            
            inter_pt[2]                   
        ])
        self.ohe_pole_angle_points = [top_pt, inter_pt, track_pt]
        self.add_line_between_points(top_pt, inter_pt, "Red", show_label=False)
        self.add_line_between_points(inter_pt, track_pt, "Red", show_label=False)

        # --- Step 2: Compute height of OHE pole above rail level ---
        height = abs(top_pt[2] - inter_pt[2])

        # --- Step 3: Compute 3D angle between pole and horizontal plane ---
        v_pole = top_pt - inter_pt                         # Pole vector
        normal_plane = np.array([0, 0, 1])                 # Horizontal plane normal

        dot = np.dot(v_pole, normal_plane)
        mag_v = np.linalg.norm(v_pole)
        mag_n = np.linalg.norm(normal_plane)

        if mag_v == 0 or mag_n == 0:
            QMessageBox.warning(None, "Error", "Invalid points for angle measurement.")
            return

        # Angle between pole and plane's normal (i.e., from vertical)
        angle_with_normal = np.arccos(np.clip(dot / (mag_v * mag_n), -1.0, 1.0))
        angle_with_normal_deg = np.degrees(angle_with_normal)

        # Angle between pole and horizontal plane
        angle_deg = 90.0 - angle_with_normal_deg

        self.ohe_pole_angle_with_rail = angle_deg
        self.ohe_pole_height_meters = height

        # --- Step 4: Display results ---
        self.output_list.addItem(f"Angle between OHE pole and rail level = {angle_deg:.2f}°")
        self.output_list.addItem(f"Height of OHE pole above rail level = {height:.2f} m")

        # --- Show angle label at intersection point (B) ---
        label_text = f"{angle_deg:.1f}°"
        self.add_text_label(inter_pt, label_text, color="Blue", scale=0.4) 

        self.vtk_widget.GetRenderWindow().Render()
    
# =======================================================================================================================================
# Define function for the All Angle Measurements:
    def process_all_angles(self):
        """Measure the angle formed by three picked points on the pole."""
        if not hasattr(self, 'measurement_points') or len(self.measurement_points) != 3:
            return

        # Get 3 points
        p1, p2, p3 = self.measurement_points[:3]
        self.plotting_active = False

        self.all_angles_points = self.measurement_points[:3]

        # Convert to numpy arrays
        v1 = np.array(p1) - np.array(p2)
        v2 = np.array(p3) - np.array(p2)    

        # Compute dot product and angle
        dot = np.dot(v1, v2)
        mag = np.linalg.norm(v1) * np.linalg.norm(v2)
        if mag == 0:
            QMessageBox.warning(None, "Error", "Invalid points for angle measurement.")
            return

        angle_rad = math.acos(dot / mag)
        angle_deg = math.degrees(angle_rad)

        self.stored_all_angles = angle_deg

        # Optionally, you can print or show the angle
        self.output_list.addItem(f"Angle at B = {angle_deg:.2f}°")

        # --- Show angle label at intersection point (B) ---
        label_text = f"{angle_deg:.1f}°"
        self.add_text_label(p2, label_text, color="Blue", scale=0.4)

        self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================    
# Define function for the Defect Angle Measurements:
    def process_defect_angle(self):
        """Measure defect angle between two rail points (left and right)."""
        if not hasattr(self, "measurement_points") or len(self.measurement_points) < 2:
            # QMessageBox.warning(None, "Error", "Please select two points on the rails.")
            return

        p1, p2 = self.measurement_points[:2]
        self.plotting_active = False

        # Identify top and bottom points based on Z
        if p1[2] >= p2[2]:
            top_point = p1
            bottom_point = p2
        else:
            top_point = p2
            bottom_point = p1

        # Create new point directly below the top point at bottom_point’s Z
        new_point = [top_point[0], top_point[1], bottom_point[2]]

        self.defect_angle_points = [top_point, bottom_point, new_point]

        # --- Draw geometry ---
        self.add_sphere_marker(top_point, label="A", radius=0.07, color="Red")
        self.add_sphere_marker(bottom_point, label="B", radius=0.07, color="Green")        
        self.add_sphere_marker(new_point, label="C", radius=0.07, color="Red")
        self.add_line_between_points(top_point, bottom_point, "Green", show_label=False)  # PQ
        self.add_line_between_points(top_point, new_point, "Green", show_label=False)      # PC (vertical)
        self.add_line_between_points(bottom_point, new_point, "Green", show_label=False)   # QC (base)

        # --- Distance helper ---
        def dist(a, b):
            return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

        PQ = dist(top_point, bottom_point)   # Hypotenuse
        QC = dist(bottom_point, new_point)   # Adjacent (horizontal)
        PC = dist(top_point, new_point)      # Opposite (vertical)

        # --- Calculate angle at the lower point (Q) ---
        if PQ == 0:
            QMessageBox.warning(None, "Error", "Invalid measurement: both points are identical.")
            return

        cos_theta = QC / PQ
        cos_theta = max(-1.0, min(1.0, cos_theta))  # Clamp for safety
        theta_rad = math.acos(cos_theta)
        theta_deg = math.degrees(theta_rad)

        self.defect_angle = theta_deg

        # --- Display result ---
        self.output_list.addItem(f"Defect Angle at Lower Rail point B = {theta_deg:.2f}°")

        # --- Show angle label slightly above lower point ---
        label_text = f"{theta_deg:.2f}°"
        label_pos = (
            bottom_point[0],
            bottom_point[1],
            bottom_point[2] + 0.2  # small Z offset
        )
        self.add_text_label(label_pos, label_text, color="Blue", scale=0.8)

        # --- Store this measurement persistently ---
        if not hasattr(self, "all_defect_angle_measurements"):
            self.all_defect_angle_measurements = []

        self.all_defect_angle_measurements.append({
            "points": {
                "top_point": list(map(float, top_point)),
                "bottom_point": list(map(float, bottom_point)),
                "vertical_point": list(map(float, new_point))
            },
            "angle_deg": float(theta_deg),
            "label": f"{theta_deg:.2f}°"
        })

        self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================    
# Define function for the Complete Curve Measurements:
    def complete_curve(self):
        self.plotting_active = False

        if len(self.measurement_points) < 3:
            return
        
        # Connect last point to first point
        p1 = self.measurement_points[-1]
        p2 = self.measurement_points[0]

        if self.current_measurement == "distance_between_catenary_to_raillevel":
            self.last_curve_points = self.measurement_points
        elif self.current_measurement == "distance_between_contact_to_raillevel":
            self.last_curve_contact_rail_points = self.measurement_points
        else:
            self.last_catenary_contact_curve_points = self.measurement_points

        # Only add the line if it doesn't already exist
        line_exists = False
        for actor in self.measurement_actors:
            if isinstance(actor.GetMapper().GetInput(), vtk.vtkLineSource):
                pos1 = actor.GetMapper().GetInput().GetPoint(0)
                pos2 = actor.GetMapper().GetInput().GetPoint(1)
                if (np.allclose(pos1, p1) and np.allclose(pos2, p2)) or \
                (np.allclose(pos1, p2) and np.allclose(pos2, p1)):
                    line_exists = True
                    break
        
        if not line_exists:
            self.add_line_between_points(p1, p2, "Purple")

        # --- Prepare spline points ---
        pts = np.array(self.measurement_points, dtype=float)
        vtk_pts = vtk.vtkPoints()
        for p in pts:
            vtk_pts.InsertNextPoint(float(p[0]), float(p[1]), float(p[2]))

        spline = vtk.vtkParametricSpline()
        spline.SetPoints(vtk_pts)

        func_src = vtk.vtkParametricFunctionSource()
        func_src.SetParametricFunction(spline)
        func_src.SetUResolution(max(200, 50 * len(pts)))
        func_src.Update()

        # Render the spline
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(func_src.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.5, 0.0, 0.8)  # purple
        actor.GetProperty().SetLineWidth(3)
        self.renderer.AddActor(actor)
        self.measurement_actors.append(actor)

        # Sample spline points
        out_pd = func_src.GetOutput()
        n_sample = out_pd.GetPoints().GetNumberOfPoints()
        sampled = np.zeros((n_sample, 3), dtype=float)
        for i in range(n_sample):
            sampled[i] = out_pd.GetPoints().GetPoint(i)

        # Baseline vector (horizontal only)
        baseline_vec = pts[-1] - pts[0]
        baseline_vec[2] = 0
        baseline_vec = baseline_vec / np.linalg.norm(baseline_vec)
        baseline_len = np.linalg.norm(pts[-1] - pts[0])

        # --- Helpers ---
        def safe_normalize(v):
            n = np.linalg.norm(v)
            return v / n if n > 1e-12 else v

        def project_to_vertical_plane(vec, base_vec):
            h = base_vec.copy()
            h[2] = 0
            h = safe_normalize(h)
            return vec - np.dot(vec, h) * h

        def point_at_horizontal_offset(sampled, baseline_vec, start_point, offset):
            baseline_dir_h = baseline_vec.copy()
            baseline_dir_h[2] = 0
            baseline_dir_h = safe_normalize(baseline_dir_h)

            target_xy = start_point[:2] + baseline_dir_h[:2] * offset
            dists = np.linalg.norm(sampled[:, :2] - target_xy, axis=1)
            idx = np.argmin(dists)

            if idx == 0:
                t_vec = sampled[1] - sampled[0]
            elif idx == len(sampled) - 1:
                t_vec = sampled[-1] - sampled[-2]
            else:
                t_vec = sampled[idx + 1] - sampled[idx - 1]
            return safe_normalize(t_vec), sampled[idx], idx

        # --- Tangent and angle at start ---
        t_start, P_offset, idx_start = point_at_horizontal_offset(sampled, baseline_vec, pts[0], 2.0)
        t_start_proj = project_to_vertical_plane(t_start, baseline_vec)
        if np.dot(t_start_proj, baseline_vec) < 0:
            t_start_proj *= -1
        interior_start = np.degrees(np.arctan2(abs(t_start_proj[2]), np.linalg.norm(t_start_proj[:2])))
        interior_start = np.clip(interior_start, 0.0, 90.0)
        exterior_start = 90.0 - interior_start

        # --- Tangent and angle at end ---
        t_end, U_offset, idx_end = point_at_horizontal_offset(sampled[::-1], -baseline_vec, pts[-1], 2.0)
        idx_end = len(sampled) - 1 - idx_end
        t_end = safe_normalize(sampled[idx_end+1] - sampled[idx_end-1])
        t_end_proj = project_to_vertical_plane(t_end, baseline_vec)
        if np.dot(t_end_proj, -baseline_vec) < 0:
            t_end_proj *= -1
        interior_end = np.degrees(np.arctan2(abs(t_end_proj[2]), np.linalg.norm(t_end_proj[:2])))
        interior_end = np.clip(interior_end, 0.0, 90.0)
        exterior_end = 90.0 - interior_end

        # --- Output angles ---
        self.output_list.addItem(
            f"Start point: Interior = {interior_start:.2f}°, Exterior = {exterior_start:.2f}°"
        )
        self.output_list.addItem(
            f"End point:   Interior = {interior_end:.2f}°, Exterior = {exterior_end:.2f}°"
        )
        self.output_list.addItem(f"Distance between two OHE poles = {baseline_len:.2f} m")

        # --- Show angle label at intersection point (B) ---
        label_text = f"{interior_start:.1f}°"
        self.add_text_label(p1, label_text, color="Blue", scale=0.4) 

        label_text = f"{interior_end:.1f}°"
        self.add_text_label(p2, label_text, color="Blue", scale=0.4) 

        # Store last computed values for saving
        self.last_sampled_points = sampled.tolist()
        self.last_interior_start = interior_start
        self.last_exterior_start = exterior_start
        self.last_interior_end = interior_end
        self.last_exterior_end = exterior_end
        self.last_baseline_len = baseline_len
        self.last_angle_labels = [
        {"pos": list(map(float, p1)), "text": f"{interior_start:.1f}°"},
        {"pos": list(map(float, p2)), "text": f"{interior_end:.1f}°"}]

        self.vtk_widget.GetRenderWindow().Render()

        if self.current_measurement == 'distance_between_catenary_to_raillevel':

            if not hasattr(self, "horizontal_points") or len(self.horizontal_points) < 2:
                return
        
            # --- Extract picked points ---
            # Assuming rail_points = [(xL, yL, zL), (xR, yR, zR)]
            left_point = self.horizontal_points[0]
            right_point = self.horizontal_points[1]

            self.left_point = left_point
            self.right_point = right_point

            x1, y1, z1 = left_point
            x2, y2, z2 = right_point

            # --- Step 3: Mean Top of Rail (MTOR) ---
            z_mean = (z1 + z2) / 2.0
            baseline_z = z_mean
            self.OHE_baseline_z = baseline_z  # Store for later use

            self.output_list.addItem(f"Rail heights: Left={z1:.2f}, Right={z2:.2f}, MTOR={z_mean:.2f}")

            self.process_height_catenary_contact_wire_to_rail(sampled, pts, baseline_len, baseline_z)

        elif self.current_measurement == 'distance_between_contact_to_raillevel':

            if not hasattr(self, "horizontal_points") or len(self.horizontal_points) < 2:
                return
        
            # --- Extract picked points ---
            # Assuming rail_points = [(xL, yL, zL), (xR, yR, zR)]
            left_point = self.horizontal_points[0]
            right_point = self.horizontal_points[1]

            x1, y1, z1 = left_point
            x2, y2, z2 = right_point

            self.left_point = left_point
            self.right_point = right_point

            # --- Step 3: Mean Top of Rail (MTOR) ---
            z_mean = (z1 + z2) / 2.0
            baseline_z = z_mean
            self.OHE_baseline_z = baseline_z  # Store for later use

            self.output_list.addItem(f"Rail heights: Left={z1:.2f}, Right={z2:.2f}, MTOR={z_mean:.2f}")

            self.process_height_catenary_contact_wire_to_rail(sampled, pts, baseline_len, baseline_z)

        else:
            if not hasattr(self, 'contact_wire_points') or len(self.contact_wire_points) < 2:
                QMessageBox.warning(None, "Error", "Please select at least two points on the contact wire.")    
                return
            contact_wire_points = self.contact_wire_points
            self.process_elevation_between_catenary_contact(sampled, pts, contact_wire_points)

# =======================================================================================================================================    
# Define function for the Height of Catenary/Contact Wire above Rail level:
    def process_height_catenary_contact_wire_to_rail(self, sampled, pts, baseline_len, baseline_z):
        # --- Visualization ---
        scale = max(baseline_len * 0.2, 1.0)
        vertical_down = np.array([0, 0, -1])

        # --- Draw vertical lines snapping to OHE baseline height ---
        OHE_baseline_z = baseline_z  

        # First point vertical line to baseline
        p0_top = pts[0]
        p0_bottom = np.array([p0_top[0], p0_top[1], OHE_baseline_z])
        self.add_line_between_points(p0_top, p0_bottom, 'blue')

        # Last point vertical line to baseline
        pN_top = pts[-1]
        pN_bottom = np.array([pN_top[0], pN_top[1], OHE_baseline_z])
        self.add_line_between_points(pN_top, pN_bottom,'blue')

        # --- Connect baseline foot points with red line ---
        self.add_line_between_points(p0_bottom, pN_bottom, 'red')  # red line connecting baseline points

        # Create corresponding bottom points for each spline point
        bottom_points = sampled.copy()
        bottom_points[:, 2] = OHE_baseline_z  # flatten all to baseline z

        # Build vtk points
        vtk_points = vtk.vtkPoints()
        for p_top, p_bottom in zip(sampled, bottom_points):
            vtk_points.InsertNextPoint(*p_top)
            vtk_points.InsertNextPoint(*p_bottom)

        # Create quads (pairs between consecutive spline segments)
        quads = vtk.vtkCellArray()
        for i in range(len(sampled) - 1):
            quad = vtk.vtkQuad()
            # top i, bottom i, bottom i+1, top i+1
            quad.GetPointIds().SetId(0, 2*i)
            quad.GetPointIds().SetId(1, 2*i + 1)
            quad.GetPointIds().SetId(2, 2*(i+1) + 1)
            quad.GetPointIds().SetId(3, 2*(i+1))
            quads.InsertNextCell(quad)

        # Create polydata and actor
        plane_polydata = vtk.vtkPolyData()
        plane_polydata.SetPoints(vtk_points)
        plane_polydata.SetPolys(quads)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(plane_polydata)

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0, 1, 0)      # green
        actor.GetProperty().SetOpacity(0.4)        # semi-transparent
        actor.GetProperty().SetEdgeVisibility(1)
        actor.GetProperty().SetEdgeColor(1, 1, 1)  # white border

        self.renderer.AddActor(actor)
        self.measurement_actors.append(actor)
        self.vtk_widget.GetRenderWindow().Render()

        # --- Vertical height measurement every 3m along baseline ---

        # Compute baseline direction and length
        baseline_vec = pN_bottom - p0_bottom
        baseline_len = np.linalg.norm(baseline_vec)
        baseline_dir = baseline_vec / baseline_len

        # Step size (3 meters)
        step = 3.0
        n_steps = int(baseline_len // step) + 1

        self.output_list.addItem("---- Vertical Height Measurements (every 3m) ----")

        for i in range(n_steps + 1):
            dist = i * step
            if dist > baseline_len:
                dist = baseline_len
            
            # Point on the baseline
            base_pt = p0_bottom + baseline_dir * dist

            # Find nearest spline point vertically above (same XY)
            xy_dists = np.linalg.norm(sampled[:, :2] - base_pt[:2], axis=1)
            idx = np.argmin(xy_dists)
            spline_pt = sampled[idx]

            # Create vertical line between them
            self.add_line_between_points(base_pt, spline_pt, 'yellow')  # yellow line

            # Calculate vertical height
            height = spline_pt[2] - base_pt[2]

            # Log the result
            self.output_list.addItem(
                f"At {dist:.2f} m → Height = {height:.3f} m (Spline Z={spline_pt[2]:.3f}, Base Z={base_pt[2]:.3f})"
            )

        self.output_list.addItem("----------------------------------------------")
        self.vtk_widget.GetRenderWindow().Render()

        # Hide Complete curve button
        self.complete_curve_button.setVisible(False)
        self.complete_curve_button.setStyleSheet("")
        self.vtk_widget.GetRenderWindow().Render()
        
# =======================================================================================================================================    
# Define function for the Elevation between Catenary and Contact Wire:
    def process_elevation_between_catenary_contact(self, sampled, pts, contact_wire_points):

        # --- Prepare and draw contact wire spline (lower) ---
        contact_pts = np.array(contact_wire_points, dtype=float)
        vtk_pts = vtk.vtkPoints()
        for p in contact_pts:
            vtk_pts.InsertNextPoint(*map(float, p))

        spline = vtk.vtkParametricSpline()
        spline.SetPoints(vtk_pts)

        func_src = vtk.vtkParametricFunctionSource()
        func_src.SetParametricFunction(spline)
        func_src.SetUResolution(max(200, 50 * len(contact_pts)))
        func_src.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(func_src.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(1.0, 0.0, 0.0)  # red
        actor.GetProperty().SetLineWidth(3)
        self.renderer.AddActor(actor)
        self.measurement_actors.append(actor)

        # --- Extract sampled contact points ---
        out_pd = func_src.GetOutput()
        n_sample = out_pd.GetPoints().GetNumberOfPoints()
        contact_sampled = np.array([out_pd.GetPoints().GetPoint(i) for i in range(n_sample)])

        # --- Find first and last matching verticals between measured and contact splines ---
        def find_closest_spline_point(point, sampled_points):
            xy_dists = np.linalg.norm(sampled_points[:, :2] - point[:2], axis=1)
            return sampled_points[np.argmin(xy_dists)]

        p0_top, pN_top = pts[0], pts[-1]
        p0_bottom = find_closest_spline_point(p0_top, contact_sampled)
        pN_bottom = find_closest_spline_point(pN_top, contact_sampled)

        self.add_line_between_points(p0_top, p0_bottom, 'blue', show_label=False)
        self.add_line_between_points(pN_top, pN_bottom, 'blue', show_label=False)

        # --- Helper: resample spline to fixed number of samples ---
        def resample_spline(points, n_samples):
            distances = np.cumsum(np.r_[0, np.linalg.norm(np.diff(points, axis=0), axis=1)])
            total_length = distances[-1]
            new_distances = np.linspace(0, total_length, n_samples)
            return np.array([np.interp(new_distances, distances, points[:, i]) for i in range(3)]).T

        upper_pts = np.array(sampled)
        lower_pts = np.array(contact_sampled)

        # --- Trim both splines between the two blue vertical boundaries ---
        p0_top, pN_top = pts[0], pts[-1]  # from earlier
        p0_bottom = find_closest_spline_point(p0_top, contact_sampled)
        pN_bottom = find_closest_spline_point(pN_top, contact_sampled)

        def crop_spline_between(points, start_pt, end_pt):
            """Keep only points lying between start and end along spline length"""
            dist_all = np.cumsum(np.r_[0, np.linalg.norm(np.diff(points, axis=0), axis=1)])
            total_len = dist_all[-1]
            proj = np.dot(points - points[0], (end_pt - start_pt) / np.linalg.norm(end_pt - start_pt))
            valid = (proj >= 0) & (proj <= np.linalg.norm(end_pt - start_pt))
            return points[valid]

        upper_pts = crop_spline_between(upper_pts, p0_top, pN_top)
        lower_pts = crop_spline_between(lower_pts, p0_bottom, pN_bottom)

        # Resample lower spline to match upper spline
        if len(lower_pts) != len(upper_pts):
            lower_pts = resample_spline(lower_pts, len(upper_pts))

        n_pairs = len(upper_pts)

        # --- Build purple semi-transparent plane between splines ---
        vtk_points = vtk.vtkPoints()
        for up, low in zip(upper_pts, lower_pts):
            vtk_points.InsertNextPoint(*up)
            vtk_points.InsertNextPoint(*low)

        quads = vtk.vtkCellArray()
        for i in range(n_pairs - 1):
            quad = vtk.vtkQuad()
            quad.GetPointIds().SetId(0, 2*i)
            quad.GetPointIds().SetId(1, 2*i + 1)
            quad.GetPointIds().SetId(2, 2*(i+1) + 1)
            quad.GetPointIds().SetId(3, 2*(i+1))
            quads.InsertNextCell(quad)

        plane_polydata = vtk.vtkPolyData()
        plane_polydata.SetPoints(vtk_points)
        plane_polydata.SetPolys(quads)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(plane_polydata)

        plane_actor = vtk.vtkActor()
        plane_actor.SetMapper(mapper)
        plane_actor.GetProperty().SetColor(0.7, 0.0, 0.9)  # purple
        plane_actor.GetProperty().SetOpacity(0.4)
        plane_actor.GetProperty().SetEdgeVisibility(1)

        self.renderer.AddActor(plane_actor)
        self.measurement_actors.append(plane_actor)

        # --- Vertical height measurement every 3m between contact and main splines ---
        p0_bottom, pN_bottom = lower_pts[0], lower_pts[-1]
        baseline_vec = pN_bottom - p0_bottom
        baseline_len = np.linalg.norm(baseline_vec)
        baseline_dir = baseline_vec / baseline_len

        step = 3.0
        n_steps = int(baseline_len // step) + 1

        self.output_list.addItem("---- Vertical Heights between Contact & Main Wire (every 3m) ----")

        for i in range(n_steps + 1):
            dist = min(i * step, baseline_len)
            base_pt = p0_bottom + baseline_dir * dist
            xy_base = base_pt[:2]  # (X, Y) stays fixed for vertical line

            # --- find points on both splines closest in XY to this position ---
            idx_low = np.argmin(np.linalg.norm(lower_pts[:, :2] - xy_base, axis=1))
            idx_up = np.argmin(np.linalg.norm(upper_pts[:, :2] - xy_base, axis=1))

            # take X, Y from baseline but Z from spline points
            z_low = lower_pts[idx_low][2]
            z_up = upper_pts[idx_up][2]

            # construct perfectly vertical points
            lower_pt = np.array([xy_base[0], xy_base[1], z_low])
            upper_pt = np.array([xy_base[0], xy_base[1], z_up])

            height = z_up - z_low

            self.add_line_between_points(lower_pt, upper_pt, 'orange')
            self.output_list.addItem(
                f"At {dist:.2f} m → Height = {height:.3f} m "
                f"(Upper Z={z_up:.3f}, Lower Z={z_low:.3f})"
            )

        self.output_list.addItem("------------------------------------------------------------")

        # Hide "Complete Curve" button
        self.complete_curve_button.setVisible(False)
        self.complete_curve_button.setStyleSheet("")
        self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================
    def process_vertical_line_measurement(self):
        """Process vertical line measurement and calculate volume for round pillar if applicable."""
        if len(self.measurement_points) != 2:
            return

        point_a = self.measurement_points[0]
        point_b = self.measurement_points[1]
        self.vertical_points = [point_a, point_b]

        # Calculate vertical height in meters (difference in Z coordinates)
        self.vertical_height_meters = np.linalg.norm(point_b - point_a)
        self.plotting_active = False

        # Get current units
        units_suffix, conversion_factor = self.get_current_units()
        height = self.vertical_height_meters * conversion_factor

        # Draw the vertical line and store the actor
        self.main_line_actor = self.add_line_between_points(point_a, point_b, "Red", f"AB={height:.2f}{units_suffix}")
        
        # Store reference to the distance label actor (the last added actor)
        if self.measurement_actors:
            self.distance_label_actor = self.measurement_actors[-1]

        self.point_a_actor = self.add_sphere_marker(point_a, "A")
        self.point_b_actor = self.add_sphere_marker(point_b, "B")

        # Output the height
        self.output_list.addItem(f"Vertical Height AB = {height:.2f} {units_suffix}")

        # Check if point B is on baseline
        on_baseline = False
        if hasattr(self, 'baseline_actors') and self.baseline_actors:
            baseline_actor = self.baseline_actors[0]
            bounds = baseline_actor.GetBounds()
            baseline_z = bounds[4]  # Minimum Z of the baseline plane
            if abs(point_b[2] - baseline_z) < 0.001:  # Account for floating point precision
                on_baseline = True

        # # --- Store multiple lines ---
        # if not hasattr(self, "all_vertical_lines"):
        #     self.all_vertical_lines = []

        # self.all_vertical_lines.append({
        #     "points": {"A": point_a.tolist(), "B": point_b.tolist()},
        #     "height": height,
        #     "unit": units_suffix,
        #     "on_baseline": on_baseline
        # })

        # Initialize line_data if it doesn't exist
        if not hasattr(self, 'line_data'):
            self.line_data = {}

        # Store basic line data
        self.line_data = {
            'length': height,
            'points': {
                'A': point_a,
                'B': point_b
            },
            'on_baseline': on_baseline
        }
        # Store measurement data
        self.measurement_widget.measurement_data.update({
            'height': height,
            # 'volume': volume,
            # 'outer_surface': outer_surface,
            'points': {
            'A': point_a,
            'B': point_b,
            'offsets': self.line_data.get('offsets', {}),  # Safely get offsets if they exist
            'on_baseline': on_baseline
        }
        })

        self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================
    def process_horizontal_line_measurement(self):
        """Process horizontal line measurement with two points"""
        if len(self.measurement_points) != 2:
            return
            
        point_p = self.measurement_points[0]
        point_q = self.measurement_points[1]
        self.horizontal_points = [point_p, point_q]
        
        # Calculate distance between P and Q
        distance_meters = np.linalg.norm(point_q - point_p)
        self.horizontal_length_meters = distance_meters
        self.plotting_active = False

        units_suffix, conversion_factor = self.get_current_units()
        distance = distance_meters * conversion_factor
        
        # Draw main horizontal line (red) and store reference
        self.horizontal_line_actor = self.add_line_between_points(point_p, point_q, "Red", f"PQ={distance:.2f}{units_suffix}")

        # Store reference to the distance label actor (the last added actor)
        if self.measurement_actors:
            self.horizontal_distance_label_actor = self.measurement_actors[-1]
        
        # Add point markers and store references
        self.point_p_actor = self.add_sphere_marker(point_p, "P")
        self.point_q_actor = self.add_sphere_marker(point_q, "Q")

        # # --- Store multiple lines ---
        # if not hasattr(self, "all_horizontal_lines"):
        #     self.all_horizontal_lines = []

        # self.all_horizontal_lines.append({
        #     "points": {"P": point_p.tolist(), "Q": point_q.tolist()},
        #     "length": distance,
        #     "unit": units_suffix
        # })
        
        # Initialize line_data if it doesn't exist
        if not hasattr(self, 'line_data'):
            self.line_data = {}

        # Store basic line data
        self.line_data = {
            'length': distance,
            'points': {
                'P': point_p,
                'Q': point_q
            }
        }
        
        # Output results
        self.output_list.addItem(f"Horizontal line PQ length: {distance:.2f} {units_suffix}")

        # Update measurement widget
        self.measurement_widget.measurement_data = {
            'length': distance,
            'points': {
                'P': point_p,
                'Q': point_q,
                'offsets': self.line_data.get('offsets', {})  # Safely get offsets if they exist
            }
        }
        self.measurement_widget.update()

# =======================================================================================================================================
# Define the function for the Measurement Line of 3 Points
    def process_measurement_line(self):
        """Process measurement line with three points (M, N, O) at same Z-level"""
        if len(self.measurement_line_points) != 3:
            return
            
        # Ensure all points are at same Z-level (use first point's Z)
        base_z = self.measurement_line_points[0][2]
        for i in range(1, 3):
            self.measurement_line_points[i][2] = base_z
        
        point_m = self.measurement_line_points[0]
        point_n = self.measurement_line_points[1]
        point_o = self.measurement_line_points[2]
        
        # Calculate distances
        mn_distance_meters = np.linalg.norm(point_n - point_m)
        no_distance_meters = np.linalg.norm(point_o - point_n)
        # mo_distance_meters = np.linalg.norm(point_o - point_m)
        
        # Calculate area (rectangle)
        area_meters = mn_distance_meters * no_distance_meters
        
        # Get current units
        units_suffix, conversion_factor = self.get_current_units()
        
        # Convert measurements
        mn_distance = mn_distance_meters * conversion_factor
        no_distance = no_distance_meters * conversion_factor
        area = area_meters * (conversion_factor ** 2)
        
        # Store measurements for later volume calculation
        self.measurement_widget.measurement_data = {
            'mn_distance': mn_distance,
            'no_distance': no_distance,
            'area_meters': area_meters,
            'perimeter_meters': 2 * (mn_distance_meters + no_distance_meters),
            'points': {
                'M': point_m,
                'N': point_n,
                'O': point_o
            }
        }
        
        # Draw lines and add labels
        self.add_line_between_points(point_m, point_n, "Red", f"MN={mn_distance:.2f}{units_suffix}")
        self.add_line_between_points(point_n, point_o, "Red", f"NO={no_distance:.2f}{units_suffix}")
        
        # Add area label at centroid
        centroid = (point_m + point_n + point_o) / 3
        self.add_text_label(centroid, f"Area={area:.2f} sq{units_suffix.replace('m','')}", "White")
        
        # Output results
        self.output_list.addItem(f"Measurement Line:")
        self.output_list.addItem(f"MN = {mn_distance:.3f} {units_suffix}")
        self.output_list.addItem(f"NO = {no_distance:.3f} {units_suffix}")
        self.output_list.addItem(f"Area = {area:.3f} sq{units_suffix.replace('m','')}")
        
        # Prompt for vertical line
        self.output_list.addItem("Now select 'Vertical Line' to measure height")

# =======================================================================================================================================
    def process_measurement_with_vertical(self):
        """Process combined measurement line and vertical line for volume calculation"""
        if (not hasattr(self, 'measurement_line_points') or len(self.measurement_line_points) < 3 or 
            len(self.measurement_points) < 2):
            return
            
        M, N, O = self.measurement_line_points[:3]
        A, B = self.measurement_points[:2]
        
        # Calculate distances in meters
        MN_meters = np.linalg.norm(N - M)
        NO_meters = np.linalg.norm(O - N)
        AB_meters = np.linalg.norm(B - A)
        
        # Convert to current units
        units_suffix, conversion_factor = self.get_current_units()
        MN = MN_meters * conversion_factor
        NO = NO_meters * conversion_factor
        AB = AB_meters * conversion_factor
        
        # Calculate surface area and volume
        surface_area_meters = MN_meters * NO_meters
        volume_meters = surface_area_meters * AB_meters
        
        # Calculate perimeter for outer surface area
        perimeter_meters = 2 * (MN_meters + NO_meters)
        outer_surface_meters = perimeter_meters * AB_meters
        
        # Convert measurements
        if units_suffix == "cm":
            surface_area = surface_area_meters * 10000
            volume = volume_meters * 1000000
            outer_surface = outer_surface_meters * 10000
            area_suffix = "square cm"
            volume_suffix = "cubic cm"
        elif units_suffix == "mm":
            surface_area = surface_area_meters * 1000000
            volume = volume_meters * 1000000000
            outer_surface = outer_surface_meters * 1000000
            area_suffix = "square mm"
            volume_suffix = "cubic mm"
        else:
            surface_area = surface_area_meters
            volume = volume_meters
            outer_surface = outer_surface_meters
            area_suffix = "square m"
            volume_suffix = "cubic meter"
        
        # Draw all lines with labels
        self.add_line_between_points(M, N, "Red", f"MN={MN:.2f}{units_suffix}")
        self.add_line_between_points(N, O, "Red", f"NO={NO:.2f}{units_suffix}")
        self.add_line_between_points(A, B, "Blue", f"AB={AB:.2f}{units_suffix}")
        
        # Add point markers with labels
        self.add_sphere_marker(M, "M")
        self.add_sphere_marker(N, "N")
        self.add_sphere_marker(O, "O")
        self.add_sphere_marker(A, "A")
        self.add_sphere_marker(B, "B")
        
        # Calculate angles to ensure lines are straight (90° between MN and NO)
        vector_MN = N - M
        vector_NO = O - N
        angle_MNO = degrees(acos(np.dot(vector_MN, vector_NO) / (np.linalg.norm(vector_MN) * np.linalg.norm(vector_NO))))
        
        # Output results
        self.output_list.addItem("Measurement Results:")
        self.output_list.addItem(f"Distance MN = {MN:.3f} {units_suffix}")
        self.output_list.addItem(f"Distance NO = {NO:.3f} {units_suffix}")
        self.output_list.addItem(f"Height AB = {AB:.3f} {units_suffix}")
        self.output_list.addItem(f"Angle at N = {angle_MNO:.2f}°")
        self.output_list.addItem(f"Base Area = {surface_area:.3f} {area_suffix}")
        self.output_list.addItem(f"Outer Surface Area = {outer_surface:.3f} {area_suffix}")
        self.output_list.addItem(f"Volume = {volume:.3f} {volume_suffix}")
        self.output_list.addItem("Click 'Presized' to create accurate vertical measurement")
        
        # Store measurement data
        self.measurement_widget.measurement_data = {
            'lengths': {
                'MN': MN,
                'NO': NO,
                'AB': AB
            },
            'angle': angle_MNO,
            'area': surface_area,
            'outer_surface': outer_surface,
            'volume': volume
        }
        self.measurement_widget.update()

# =======================================================================================================================================        
    def complete_polygon(self):
        self.plotting_active = False
        if self.current_measurement != 'polygon' and self.current_measurement != 'round_pillar_polygon':
            return
            
        if len(self.measurement_points) < 3:
            self.output_list.addItem("Need at least 3 points to complete a polygon")
            return
            
        # Connect last point to first point
        p1 = self.measurement_points[-1]
        p2 = self.measurement_points[0]
        
        # Only add the line if it doesn't already exist
        line_exists = False
        for actor in self.measurement_actors:
            if isinstance(actor.GetMapper().GetInput(), vtk.vtkLineSource):
                pos1 = actor.GetMapper().GetInput().GetPoint(0)
                pos2 = actor.GetMapper().GetInput().GetPoint(1)
                if (np.allclose(pos1, p1) and np.allclose(pos2, p2)) or \
                (np.allclose(pos1, p2) and np.allclose(pos2, p1)):
                    line_exists = True
                    break
        
        if not line_exists:
            if self.current_measurement == 'round_pillar_polygon':
                self.add_line_between_points(p1, p2, "Purple", show_label=False)
            else:
                self.add_line_between_points(p1, p2, "Purple")
                    
        if self.current_measurement == 'round_pillar_polygon':
            self.process_round_pillar_polygon_measurement()

        # Process the polygon measurement                                                         
        if self.current_measurement == 'polygon' :            
            self.process_polygon_measurement() 

        # Set all polygon points to red initially
        for i in range(len(self.measurement_points)):
            self.change_polygon_point_color(i, "Red")
            
        # Hide the Complete Polygon button after completing
        self.complete_polygon_button.setVisible(False)
        self.complete_polygon_button.setStyleSheet("")  # Reset to default style
            
        # Only show digging point sections if in extraction mode
        if self.extraction_check.isChecked():
            # Update the digging point input
            self.digging_point_input.polygon_points = self.measurement_points
            self.digging_point_input.setVisible(True)
            self.digging_point_input.update_polygon_point_combo()

            # Show connection sections
            self.connection_group.setVisible(True)
            self.polygon_digging_group.setVisible(True)
            
            # Update the connection dropdowns
            self.update_polygon_digging_combos()
            
            # Show surface-related buttons in Measurement section
            self.show_surface_controls_in_measurement()
                
        self.vtk_widget.GetRenderWindow().Render()
        
# =======================================================================================================================================        
# Define function for the Polygon Measurements:
    def process_polygon_measurement(self):
        """Process polygon measurement on any plane using triangulation for area calculation"""
        if len(self.measurement_points) < 3:
            return
        
        # Store polygon points and actors for saving to JSON later
        self.polygon_points = [p.tolist() for p in self.measurement_points]
        self.polygon_actors = list(self.measurement_actors)
            
        points = self.measurement_points
        n = len(points)

        # Find the best fitting plane
        centroid, normal = find_best_fitting_plane(points)
        
        # Get current units
        units_suffix, conversion_factor = self.get_current_units()

        # Project points onto the best-fit plane
        projected_points = []
        for point in points:
            v = point - centroid
            dist = np.dot(v, normal)
            projected = point - dist * normal
            projected_points.append(projected)
        
        projected_points = np.array(projected_points)

        perimeter_meters = 0.0
        # Calculate all side lengths
        lengths = []
        for i in range(n):
            p1 = projected_points[i]
            p2 = projected_points[(i+1)%n]
            length_meters = np.linalg.norm(p2 - p1)
            perimeter_meters += length_meters
            length = length_meters * conversion_factor
            label = f"{chr(65+i)}{chr(65+(i+1)%n)}"
            lengths.append((label, length))
        
        # Calculate all internal angles
        angles = []
        for i in range(n):
            a = projected_points[i]
            b = projected_points[(i+1)%n]
            c = projected_points[(i+2)%n]
            
            ba = a - b
            bc = c - b
            cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
            angle = degrees(np.arccos(cosine_angle))
            label = chr(65 + (i+1)%n)  # B, C, etc.
            angles.append((label, angle))
        
        # Calculate area using triangulation (Delaunay triangulation)
        try:
            from scipy.spatial import Delaunay
            
            # Project points to 2D plane (using the two most significant dimensions)
            abs_normal = np.abs(normal)
            if abs_normal[0] > abs_normal[1] and abs_normal[0] > abs_normal[2]:
                # Most aligned with YZ plane
                coords = projected_points[:, [1, 2]]  # Use Y and Z coordinates
            elif abs_normal[1] > abs_normal[2]:
                # Most aligned with XZ plane
                coords = projected_points[:, [0, 2]]  # Use X and Z coordinates
            else:
                # Most aligned with XY plane (default)
                coords = projected_points[:, [0, 1]]  # Use X and Y coordinates
            
            # Perform Delaunay triangulation
            tri = Delaunay(coords)
            
            # Calculate area of each triangle and sum them up
            area_meters = 0.0
            for simplex in tri.simplices:
                # Get the 3 points of the triangle
                a, b, c = projected_points[simplex]
                # Calculate area of triangle using cross product
                area_meters += 0.5 * np.linalg.norm(np.cross(b - a, c - a))
            
        except ImportError:
            # Fallback to shoelace formula if scipy not available
            self.output_list.addItem("Note: scipy not available, using shoelace formula instead")
            x = projected_points[:,0]
            y = projected_points[:,1]
            z = projected_points[:,2]
            
            # Find which plane we're most aligned with
            if abs_normal[0] > abs_normal[1] and abs_normal[0] > abs_normal[2]:
                # Most aligned with YZ plane
                area_meters = 0.5 * np.abs(np.dot(y, np.roll(z,1)) - np.dot(z, np.roll(y,1)))
            elif abs_normal[1] > abs_normal[2]:
                # Most aligned with XZ plane
                area_meters = 0.5 * np.abs(np.dot(x, np.roll(z,1)) - np.dot(z, np.roll(x,1)))
            else:
                # Most aligned with XY plane (default)
                area_meters = 0.5 * np.abs(np.dot(x, np.roll(y,1)) - np.dot(y, np.roll(x,1)))
        
        # Convert area
        if units_suffix == "cm":
            area = area_meters * 10000
            perimeter = perimeter_meters * 100
            area_suffix = "square cm"
            perimeter_suffix = "cm"
        elif units_suffix == "mm":
            area = area_meters * 1000000
            perimeter = perimeter_meters * 1000
            area_suffix = "square mm"
            perimeter_suffix = "mm"
        else:
            area = area_meters
            perimeter = perimeter_meters
            area_suffix = "square meter"
            perimeter_suffix = "m"

        # Update measurement widget data
        self.measurement_widget.measurement_data = {
            'angles': {label: angle for label, angle in angles},
            'lengths': {label: length for label, length in lengths},
            'area': area,
            'normal': normal  # Store plane normal for volume calculations
        }
        self.measurement_widget.update()
        
        # Add angle labels to the 3D view
        for i in range(n):
            a = points[i]
            b = points[(i+1)%n]
            c = points[(i+2)%n]
            angle_label = angles[i][1]
            self.add_angle_label(b, a, c, f"{angle_label:.1f}°", offset=0.8)
        
        self.polygon_area_meters = area_meters
        self.polygon_perimeter_meters = perimeter_meters
        self.output_list.addItem(f"Polygon Surface Area = {area:.2f} {area_suffix}")
        self.output_list.addItem(f"Polygon Perimeter  = {perimeter:.2f} {perimeter_suffix}")

# =======================================================================================================================================
    def process_round_pillar_polygon_measurement(self):
        """Process round pillar polygon measurement and store area and perimeter"""
        if len(self.measurement_points) < 3:
            return
        
        # Store polygon points & actors for saving
        self.round_pillar_points = [p.tolist() for p in self.measurement_points]  
        self.round_pillar_actors = list(self.measurement_actors)

        points = self.measurement_points
        n = len(points)

        # Find the best fitting plane
        centroid, normal = find_best_fitting_plane(points)
        
        # Get current units
        units_suffix, conversion_factor = self.get_current_units()

        # Project points onto the best-fit plane
        projected_points = []
        for point in points:
            v = point - centroid
            dist = np.dot(v, normal)
            projected = point - dist * normal
            projected_points.append(projected)
        
        projected_points = np.array(projected_points)

        # Calculate all side lengths and perimeter
        lengths = []
        perimeter_meters = 0.0
        for i in range(n):
            p1 = projected_points[i]
            p2 = projected_points[(i+1)%n]
            length_meters = np.linalg.norm(p2 - p1)
            perimeter_meters += length_meters
            length = length_meters * conversion_factor
            label = f"{chr(65+i)}{chr(65+(i+1)%n)}"
            lengths.append((label, length))
        
        # Calculate area using triangulation (Delaunay triangulation)
        try:
            from scipy.spatial import Delaunay
            
            # Project points to 2D plane (using the two most significant dimensions)
            abs_normal = np.abs(normal)
            if abs_normal[0] > abs_normal[1] and abs_normal[0] > abs_normal[2]:
                # Most aligned with YZ plane
                coords = projected_points[:, [1, 2]]  # Use Y and Z coordinates
            elif abs_normal[1] > abs_normal[2]:
                # Most aligned with XZ plane
                coords = projected_points[:, [0, 2]]  # Use X and Z coordinates
            else:
                # Most aligned with XY plane (default)
                coords = projected_points[:, [0, 1]]  # Use X and Y coordinates
            
            # Perform Delaunay triangulation
            tri = Delaunay(coords)
            
            # Calculate area of each triangle and sum them up
            area_meters = 0.0
            for simplex in tri.simplices:
                # Get the 3 points of the triangle
                a, b, c = projected_points[simplex]
                # Calculate area of triangle using cross product
                area_meters += 0.5 * np.linalg.norm(np.cross(b - a, c - a))
            
        except ImportError:
            # Fallback to shoelace formula if scipy not available
            self.output_list.addItem("Note: scipy not available, using shoelace formula instead")
            x = projected_points[:,0]
            y = projected_points[:,1]
            z = projected_points[:,2]
            
            # Find which plane we're most aligned with
            if abs_normal[0] > abs_normal[1] and abs_normal[0] > abs_normal[2]:
                # Most aligned with YZ plane
                area_meters = 0.5 * np.abs(np.dot(y, np.roll(z,1)) - np.dot(z, np.roll(y,1)))
            elif abs_normal[1] > abs_normal[2]:
                # Most aligned with XZ plane
                area_meters = 0.5 * np.abs(np.dot(x, np.roll(z,1)) - np.dot(z, np.roll(x,1)))
            else:
                # Most aligned with XY plane (default)
                area_meters = 0.5 * np.abs(np.dot(x, np.roll(y,1)) - np.dot(y, np.roll(x,1)))
        
        # Convert area
        if units_suffix == "cm":
            area = area_meters * 10000
            perimeter = perimeter_meters * 100
            area_suffix = "square cm"
            perimeter_suffix = "cm"
        elif units_suffix == "mm":
            area = area_meters * 1000000
            perimeter = perimeter_meters * 1000
            area_suffix = "square mm"
            perimeter_suffix = "mm"
        else:
            area = area_meters
            perimeter = perimeter_meters
            area_suffix = "square meter"
            perimeter_suffix = "meter"

        # Store measurements for later use with vertical line
        self.round_pillar_surface_area_meters = area_meters
        self.round_pillar_perimeter_meters = perimeter_meters
        
        # Update measurement widget data
        self.measurement_widget.measurement_data = {
            'angles': {},  # Not calculated for round pillar
            'lengths': {label: length for label, length in lengths},
            'area': area,
            'perimeter': perimeter,
            'area_meters': area_meters,
            'perimeter_meters': perimeter_meters,
            'normal': normal  # Store plane normal
        }
        self.measurement_widget.update()
        
        # Output results
        self.output_list.addItem(f"Round Pillar Polygon with {n} sides:")
        # for label, length in lengths:
        #     self.output_list.addItem(f"Length {label} = {length:.3f} {units_suffix}")
        self.output_list.addItem(f"Perimeter = {perimeter:.2f} {perimeter_suffix}")
        self.output_list.addItem(f"Round Piller Polygon Surface Area = {area:.2f} {area_suffix}")

# =======================================================================================================================================
    def change_polygon_point_color(self, idx, color_name):
        """Change color of specific polygon point sphere"""
        if idx < 0 or idx >= len(self.measurement_points):
            return
    
        # Find all sphere actors and check their stored point_index
        for actor in self.measurement_actors:
            if hasattr(actor, 'point_index') and actor.point_index == idx:
                actor.GetProperty().SetColor(self.colors.GetColor3d(color_name))
                break
    
        self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================
    def show_surface_controls_in_measurement(self):
        """Show surface-related controls in the Measurement section"""
        # Create a container for surface controls if it doesn't exist
        self.surface_selection_group.setVisible(True)
        
        # Update the surface combo if we have surface info
        if hasattr(self, 'surface_info') and self.surface_info:
            self.update_surface_combo()

        if not hasattr(self, 'surface_controls_container'):
            self.surface_controls_container = QWidget()
            self.surface_controls_layout = QVBoxLayout()
            self.surface_controls_container.setLayout(self.surface_controls_layout)
            
            # Surface buttons
            self.surface_button = QPushButton("Identify Surfaces")
            self.surface_button.clicked.connect(self.identify_surfaces)
            
            # Cut and Move buttons in a row
            button_row = QHBoxLayout()
            self.cut_surface_button = QPushButton("Cut Surface")
            self.cut_surface_button.clicked.connect(self.cut_surface)
            self.move_button = QPushButton("Move")
            self.move_button.clicked.connect(self.toggle_surface_movement)
            self.move_button.setCheckable(True)
            button_row.addWidget(self.cut_surface_button)
            button_row.addWidget(self.move_button)
            
            # Volume button
            self.volume_button = QPushButton("Calculate Volume")
            self.volume_button.clicked.connect(self.calculate_selected_volume)
            
            # Surface Selection Group
            self.surface_selection_group = QGroupBox("Surface Selection")
            surface_selection_layout = QHBoxLayout()
            self.surface_combo = QComboBox()
            self.surface_combo.setPlaceholderText("Select Surface")
            surface_selection_layout.addWidget(self.surface_combo)
            self.surface_ok_button = QPushButton("OK")
            self.surface_ok_button.clicked.connect(self.highlight_selected_surface)
            surface_selection_layout.addWidget(self.surface_ok_button)
            self.surface_selection_group.setLayout(surface_selection_layout)
            
            # Add widgets to layout
            self.surface_controls_layout.addWidget(self.surface_button)
            self.surface_controls_layout.addLayout(button_row)
            self.surface_controls_layout.addWidget(self.volume_button)
            self.surface_controls_layout.addWidget(self.surface_selection_group)
            
            # Add container to measurement layout
            self.measurement_layout.addWidget(self.surface_controls_container)
        
        # Show the container
        self.surface_controls_container.setVisible(True)
        self.surface_selection_group.setVisible(False)

# =======================================================================================================================================
# Define a function to make connection between the Digging point
    def toggle_connection_group(self, visible):
        """Show or hide the digging point connection group"""
        self.connection_group.setVisible(visible)
        if visible and hasattr(self, 'digging_points_info') and self.digging_points_info:
            self.update_digging_point_combos()  

# =======================================================================================================================================
# Define function for the add digging point in point cloud data :
    def add_digging_point(self, point, label=None):
        """Add a blue digging point marker at the specified position"""
        sphere = vtkSphereSource()
        sphere.SetRadius(0.8)  # Slightly larger than measurement points
        sphere.SetCenter(point[0], point[1], point[2])
    
        mapper = vtkPolyDataMapper()
        mapper.SetInputConnection(sphere.GetOutputPort())

        actor = vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(self.colors.GetColor3d("Blue"))  # Blue color for digging points

        self.renderer.AddActor(actor)
        self.measurement_actors.append(actor)

        # Get current polygon point reference
        current_poly_point = self.digging_point_input.current_polygon_point
        poly_label = chr(65 + current_poly_point)

        # Use provided label or create default
        if label is None:
            dp_count = 1
            for a in self.measurement_actors:
                if isinstance(a, vtk.vtkFollower):
                    try:
                        text_source = a.GetMapper().GetInputConnection(0, 0).GetProducer()
                        if isinstance(text_source, vtk.vtkVectorText):
                            text = text_source.GetText()
                            if text and text.startswith(f"DP{self.digging_point_input.current_digging_point+1}_"):
                                dp_count += 1
                    except:
                        continue
            label = f"DP{self.digging_point_input.current_digging_point+1}_{poly_label}{dp_count}"

        text = vtk.vtkVectorText()
        text.SetText(label)

        text_mapper = vtk.vtkPolyDataMapper()
        text_mapper.SetInputConnection(text.GetOutputPort())

        text_actor = vtk.vtkFollower()
        text_actor.SetMapper(text_mapper)
        text_actor.SetScale(0.1, 0.1, 0.1)
        text_actor.AddPosition(point[0] + 0.15, point[1] + 0.15, point[2])
        text_actor.GetProperty().SetColor(self.colors.GetColor3d("White"))

        self.renderer.AddActor(text_actor)
        text_actor.SetCamera(self.renderer.GetActiveCamera())
        self.measurement_actors.append(text_actor)

        # Store digging point info
        if not hasattr(self, 'digging_points_info'):
            self.digging_points_info = []

        self.digging_points_info.append({
            'label': label,
            'point': point,
            'poly_ref': poly_label,
            'poly_point': self.digging_point_input.polygon_points[current_poly_point] 
                        if hasattr(self.digging_point_input, 'polygon_points') and 
                            current_poly_point is not None and 
                            current_poly_point < len(self.digging_point_input.polygon_points)
                        else None
        })

        # Update connection dropdowns
        self.update_digging_point_combos()
        self.toggle_connection_group(True)
        self.update_polygon_digging_combos()

        # Render the changes
        self.vtk_widget.GetRenderWindow().Render()

        return label
    
# =======================================================================================================================================
# Function for the update digging point color:
    def update_digging_point_colors(self):
        """Update colors of selected digging points in the connection interface"""
        # First reset all digging points to blue
        for actor in self.measurement_actors:
            if isinstance(actor.GetMapper().GetInput(), vtk.vtkSphereSource):
                if actor.GetProperty().GetColor() == self.colors.GetColor3d("Yellow"):
                    actor.GetProperty().SetColor(self.colors.GetColor3d("Blue"))
    
        # Highlight selected points in yellow
        from_idx = self.from_digging_combo.currentIndex()
        to_idx = self.to_digging_combo.currentIndex()
    
        # Change 'From' point color
        if from_idx >= 0:
            from_label = self.from_digging_combo.currentText()
            self.change_digging_point_color(from_label, "Yellow")
    
        # Change 'To' point color
        if to_idx >= 0:
            to_label = self.to_digging_combo.currentText()
            self.change_digging_point_color(to_label, "Yellow")
    
        self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================
# Function for the change the digging point color:
    def change_digging_point_color(self, label, color_name):
        """Change color of a specific digging point by its label"""
        for actor in self.measurement_actors:
            if isinstance(actor, vtk.vtkActor):
                # Check if this is a sphere actor (digging point)
                if isinstance(actor.GetMapper().GetInput(), vtk.vtkSphereSource):
                    # Find its corresponding label actor
                    idx = self.measurement_actors.index(actor)
                    if idx + 1 < len(self.measurement_actors):
                        text_actor = self.measurement_actors[idx + 1]
                        if isinstance(text_actor, vtk.vtkFollower):
                            text_source = text_actor.GetMapper().GetInputConnection(0, 0).GetProducer()
                            if isinstance(text_source, vtk.vtkVectorText):
                                if text_source.GetText() == label:
                                    actor.GetProperty().SetColor(self.colors.GetColor3d(color_name))
                                    return

# =======================================================================================================================================
# Connect digging point to each other:      
    def connect_digging_points(self):
        """Connect two selected digging points with a line"""
        from_idx = self.from_digging_combo.currentIndex()
        to_idx = self.to_digging_combo.currentIndex()

        if from_idx < 0 or to_idx < 0 or from_idx == to_idx:
            return
        
        from_label = self.from_digging_combo.currentText()
        to_label = self.to_digging_combo.currentText()

        # Find the points from the stored digging points info
        from_point = None
        to_point = None
        for dp in getattr(self, 'digging_points_info', []):
            if dp['label'] == from_label:
                from_point = dp['point']
            if dp['label'] == to_label:
                to_point = dp['point']

        if from_point is None or to_point is None:
            return

        # Calculate distance between points in meters
        distance_meters = sqrt(sum((from_point - to_point) ** 2))
        
        # Convert to current units
        units_suffix, conversion_factor = self.get_current_units()
        distance = distance_meters * conversion_factor

        # Calculate angle (assuming horizontal plane)
        dx = to_point[0] - from_point[0]
        dy = to_point[1] - from_point[1]
        angle = degrees(atan2(dy, dx)) % 360  # Angle in degrees (0-360)

        # Draw green line between points with label
        self.add_line_between_points(from_point, to_point, "Green", f"{distance:.2f}{units_suffix}")

        # Reset colors after connecting
        self.update_digging_point_colors()

# =======================================================================================================================================
# Define the function for the update digging point combos:
    def update_digging_point_combos(self):
        """Update the digging point dropdowns with current digging points"""
        current_from = self.from_digging_combo.currentText()
        current_to = self.to_digging_combo.currentText()
    
        self.from_digging_combo.clear()
        self.to_digging_combo.clear()
    
        if not hasattr(self, 'digging_points_info') or not self.digging_points_info:
            return
    
        for dp in self.digging_points_info:
            self.from_digging_combo.addItem(dp['label'])
            self.to_digging_combo.addItem(dp['label'])
    
        # Restore previous selections if possible
        if current_from:
            index = self.from_digging_combo.findText(current_from)
            if index >= 0:
                self.from_digging_combo.setCurrentIndex(index)
    
        if current_to:
            index = self.to_digging_combo.findText(current_to)
            if index >= 0:
                self.to_digging_combo.setCurrentIndex(index)
    
        self.connection_group.setVisible(True)
        self.update_digging_point_colors()

# =======================================================================================================================================
# Define the function for the updating digging point colors:
    def update_digging_point_colors(self):
        """Update colors of selected digging points"""
        if not hasattr(self, 'digging_points_info') or not self.digging_points_info:
            return
        
        # Reset all digging points to blue first
        for dp_info in self.digging_points_info:
            self.change_digging_point_color(dp_info['label'], "Blue")
    
        # Highlight selected points in yellow
        from_idx = self.from_digging_combo.currentIndex()
        to_idx = self.to_digging_combo.currentIndex()
    
        if from_idx >= 0 and from_idx < len(self.digging_points_info):
            from_label = self.digging_points_info[from_idx]['label']
            self.change_digging_point_color(from_label, "Yellow")
    
        if to_idx >= 0 and to_idx < len(self.digging_points_info):
            to_label = self.digging_points_info[to_idx]['label']
            self.change_digging_point_color(to_label, "Yellow")
    
        self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================
# Define the function for the change the color of digging points while conecting the points:
    def change_digging_point_color(self, label, color_name):
        """Change color of a digging point by its label"""
        for actor in self.measurement_actors:
            if isinstance(actor.GetMapper().GetInput(), vtk.vtkSphereSource):
                try:
                    # Check if this is the digging point we're looking for
                    text_actor = None
                    idx = self.measurement_actors.index(actor)
                    if idx + 1 < len(self.measurement_actors):
                        text_actor = self.measurement_actors[idx + 1]
                        if isinstance(text_actor, vtk.vtkFollower):
                            text_source = text_actor.GetMapper().GetInputConnection(0, 0).GetProducer()
                            if isinstance(text_source, vtk.vtkVectorText):
                                if text_source.GetText() == label:
                                    actor.GetProperty().SetColor(
                                        self.colors.GetColor3d(color_name))
                                    return
                except:
                    continue

# =======================================================================================================================================
# Define the function for the upadate the polygon points measurements
    def update_polygon_point_combo(self):
        """Update the polygon point combo box with available points"""
        self.polygon_point_combo.clear()
        
        if not hasattr(self.parent, 'measurement_points') or not self.parent.measurement_points:
            return
            
        for i, point in enumerate(self.parent.measurement_points):
            label = chr(65 + i)  # A, B, C, etc.
            self.polygon_point_combo.addItem(f"Point {label}")

# =======================================================================================================================================
    def create_polygon_digging_connection_section(self):
        """Create the Polygon-Digging connection interface"""
        self.surface_combo = QComboBox()

        self.polygon_digging_group = QGroupBox("Polygon - Digging Connect")
        self.polygon_digging_layout = QFormLayout()
    
        # Polygon Point dropdown
        self.polygon_point_combo = QComboBox()
        self.polygon_point_combo.setPlaceholderText("Select Polygon Point")
    
        # Digging Point dropdown
        self.digging_point_combo = QComboBox()
        self.digging_point_combo.setPlaceholderText("Select Digging Point")
    
        # Connect button
        self.connect_button = QPushButton("Connect Points")
        self.connect_button.clicked.connect(self.connect_polygon_digging_points)
        
        # Add to layout
        self.polygon_digging_layout.addRow("Polygon Point:", self.polygon_point_combo)
        self.polygon_digging_layout.addRow("Digging Point:", self.digging_point_combo)
        self.polygon_digging_layout.addRow(self.connect_button)
    
        self.polygon_digging_group.setLayout(self.polygon_digging_layout)
        self.polygon_digging_group.setVisible(False)  # Start hidden
        self.polygon_digging_group.setFixedWidth(270)
        self.measurement_layout.addWidget(self.polygon_digging_group)

        # Clear Polygon-Digging Connect section
        if hasattr(self, 'polygon_digging_group'):
            self.polygon_point_combo.clear()
            self.digging_point_combo.clear()
            self.polygon_digging_group.setVisible(False)

# =======================================================================================================================================
# Update the polygon - digging combos:
    def update_polygon_digging_combos(self):
        """Update the polygon and digging point dropdowns"""
        self.polygon_point_combo.clear()
        self.digging_point_combo.clear()
        
        # Add polygon points
        for i, point in enumerate(self.measurement_points):
            label = chr(65 + i)  # A, B, C, etc.
            self.polygon_point_combo.addItem(f"Point {label}")
    
        # Add digging points
        if hasattr(self, 'digging_points_info') and self.digging_points_info:
            for dp in self.digging_points_info:
                self.digging_point_combo.addItem(dp['label'])
    
        # Show the connection group if we have both polygon and digging points
        if (self.polygon_point_combo.count() > 0 and 
            self.digging_point_combo.count() > 0):
            self.polygon_digging_group.setVisible(True)

# =======================================================================================================================================
# Define the function for the making connection between the polygon and digging points with help of line:
    def connect_polygon_digging_points(self):
        """Connect selected polygon point to selected digging point"""
        polygon_idx = self.polygon_point_combo.currentIndex()
        digging_label = self.digging_point_combo.currentText()

        if polygon_idx < 0 or not digging_label:
            return

        # Find the polygon point
        if polygon_idx >= len(self.measurement_points):
            return
        polygon_point = self.measurement_points[polygon_idx]

        # Find the digging point
        digging_point = None
        for dp in getattr(self, 'digging_points_info', []):
            if dp['label'] == digging_label:
                digging_point = dp['point']
                break

        if digging_point is None:
            return

        # Calculate distance in meters
        distance_meters = sqrt(sum((polygon_point - digging_point) ** 2))
        
        # Convert to current units
        units_suffix, conversion_factor = self.get_current_units()
        distance = distance_meters * conversion_factor

        # Draw line between points (using purple color to distinguish from other lines)
        self.add_line_between_points(polygon_point, digging_point, "Purple", f"{distance:.2f}{units_suffix}")

        # Output connection info
        # polygon_label = chr(65 + polygon_idx)
        # self.output_list.addItem(f"Connected Polygon Point {polygon_label} to {digging_label}")
        # self.output_list.addItem(f"Connection distance = {distance:.3f} {units_suffix}")

        # Change color of connected points temporarily
        self.change_polygon_point_color(polygon_idx, "Yellow")
        self.change_digging_point_color(digging_label, "Yellow")

        # Render changes
        self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================
    def add_point_to_round_pillar_polygon(self, point):
        """Add a point to the round pillar polygon"""
        # Ensure Z coordinate matches first point if it exists
        if len(self.measurement_points) > 0:
            point[2] = self.measurement_points[0][2]
        
        self.measurement_points.append(point)
        self.add_sphere_marker(point, chr(65 + len(self.measurement_points) - 1))
        
        # Connect points with lines if we have at least 2 points
        if len(self.measurement_points) >= 2:
            p1 = self.measurement_points[-2]
            p2 = self.measurement_points[-1]
            self.add_line_between_points(p1, p2, "Purple")
            
            # Update the measurement widget
            self.measurement_widget.add_point(QPoint(100 + 50 * (len(self.measurement_points) - 1), 
                                    100 - 50 * (len(self.measurement_points) - 1 % 2)))
            if len(self.measurement_widget.points) >= 2:
                self.measurement_widget.add_line((self.measurement_widget.points[-2], 
                                            self.measurement_widget.points[-1]))
        
        self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================
    def update_surface_combo(self):
        """Update the surface dropdown with available surfaces"""
        if not hasattr(self, 'surface_combo'):
            return  # Exit if the combo box hasn't been created yet
            
        self.surface_combo.clear()
        if hasattr(self, 'surface_info') and self.surface_info:
            for surface in self.surface_info:
                self.surface_combo.addItem(surface['label'])

# =======================================================================================================================================
# Define the function for the Identify the surfaces of 3D polygon:
    def identify_surfaces(self):
        """Identify all surfaces in the current geometry formed by polygon and digging points"""
        # Initialize surface_combo if it doesn't exist
        if not hasattr(self, 'surface_combo'):
            self.surface_combo = QComboBox()
            self.surface_combo.setPlaceholderText("Select Surface")
        
        # Clear any existing surface visuals
        self.clear_surface_visuals()
        
        # Initialize surface info list as a list of dictionaries
        self.surface_info = []
        
        # Get all polygon and digging points
        polygon_points = self.measurement_points if hasattr(self, 'measurement_points') else []
        digging_points = [dp['point'] for dp in getattr(self, 'digging_points_info', [])]
        
        if len(polygon_points) < 3 and len(digging_points) < 3:
            # self.output_list.addItem("Need at least 3 polygon or digging points to identify surfaces")
            return
        
        # 1. Identify main polygon surface (front face)
        if len(polygon_points) >= 4:
            self.surface_info.append({
                'points': polygon_points,
                'label': "Surface-Polygon",
                'color': "Red",
                'center': np.mean(polygon_points, axis=0)
            })
        
        # 2. Identify main digging points surface (back face)
        if len(digging_points) >= 4:
            self.surface_info.append({
                'points': digging_points,
                'label': "Surface-Digging",
                'color': "Blue",
                'center': np.mean(digging_points, axis=0)
            })
        
        # 3. Identify side surfaces (connecting polygon and digging points)
        side_surfaces = self._identify_side_surfaces(polygon_points, digging_points)
        self.surface_info.extend(side_surfaces)
        
        # Visualize all identified surfaces
        self._visualize_surfaces()
        
        # Update surface selection dropdown
        self.update_surface_combo()
        
        # # Show surface selection group if it exists
        if hasattr(self, 'surface_selection_group'):
            self.surface_selection_group.setVisible(True)

# =======================================================================================================================================    
# Define the function for the visualize the surfaces of 3D geometry:
    def _visualize_surfaces(self):
        """Visualize all identified surfaces with transparent colors"""
        for surface in self.surface_info:
            points = surface['points']  # This is now guaranteed to be a dictionary
            color = surface['color']
            center = surface['center']
            
            # Create polygon from points
            polygon = vtk.vtkPolygon()
            polygon.GetPointIds().SetNumberOfIds(len(points))
            for j, point in enumerate(points):
                polygon.GetPointIds().SetId(j, j)
            
            polygons = vtk.vtkCellArray()
            polygons.InsertNextCell(polygon)
            
            polygonPolyData = vtk.vtkPolyData()
            polygonPolyData.SetPoints(vtk.vtkPoints())
            for point in points:
                polygonPolyData.GetPoints().InsertNextPoint(point)
            polygonPolyData.SetPolys(polygons)
            
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(polygonPolyData)
            
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(self.colors.GetColor3d(color))
            actor.GetProperty().SetOpacity(0.3)  # Semi-transparent
            actor.GetProperty().SetEdgeColor(self.colors.GetColor3d("White"))
            actor.GetProperty().SetEdgeVisibility(1)
            actor.GetProperty().SetLineWidth(1)
            
            self.renderer.AddActor(actor)
            self.measurement_actors.append(actor)
            
            # Add label at centroid
            self.add_text_label(center, surface['label'], "White")
        
        self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================
# Define the function for identify the side surface of the 3D polygon:
    def _identify_side_surfaces(self, polygon_points, digging_points):
        """Identify side surfaces connecting polygon and digging points"""
        surfaces = []
        poly_count = len(polygon_points)
        dig_count = len(digging_points)
        colors = ["Green", "Yellow", "Cyan", "Magenta"]
        
        # If we have matching number of polygon and digging points
        if poly_count >= 2 and dig_count >= 2:
            # Create quadrilateral surfaces between corresponding points
            for i in range(max(poly_count, dig_count)):
                p1 = polygon_points[i % poly_count]
                p2 = polygon_points[(i+1) % poly_count]
                d1 = digging_points[i % dig_count]
                d2 = digging_points[(i+1) % dig_count]
                
                surfaces.append({
                    'points': [p1, p2, d2, d1],
                    'label': f"Surface-Side-{i+1}",
                    'color': colors[i % len(colors)],
                    'center': np.mean([p1, p2, d2, d1], axis=0)
                })
        else:
            # Handle cases with different number of points
            min_count = min(poly_count, dig_count)
            for i in range(min_count):
                p = polygon_points[i % poly_count]
                d = digging_points[i % dig_count]
                next_p = polygon_points[(i+1) % poly_count]
                next_d = digging_points[(i+1) % dig_count]
                
                if poly_count >= dig_count:
                    surfaces.append({
                        'points': [p, next_p, d],
                        'label': f"Surface-Tri-{i+1}",
                        'color': colors[i % len(colors)],
                        'center': np.mean([p, next_p, d], axis=0)
                    })
                else:
                    surfaces.append({
                        'points': [p, d, next_d],
                        'label': f"Surface-Tri-{i+1}",
                        'color': colors[i % len(colors)],
                        'center': np.mean([p, d, next_d], axis=0)
                    })
        
        return surfaces
    
# =======================================================================================================================================
# Define the function for the clear the surfaces
    def clear_surface_visuals(self):
        """Clear all surface labels and visualizations"""
        if not hasattr(self, 'surface_info'):
            return
        
        # Remove all surface label actors
        for actor in self.measurement_actors:
            if isinstance(actor, vtk.vtkFollower):
                try:
                    text_source = actor.GetMapper().GetInputConnection(0, 0).GetProducer()
                    if isinstance(text_source, vtk.vtkVectorText):
                        text = text_source.GetText()
                        if text and text.startswith("Surface-"):
                            self.renderer.RemoveActor(actor)
                except:
                    continue
    
        # Remove all surface actors (transparent triangles)
        for actor in self.measurement_actors:
            if isinstance(actor.GetMapper().GetInput(), vtk.vtkPolyData):
                if actor.GetProperty().GetOpacity() < 1.0:  # Our surfaces are transparent
                    self.renderer.RemoveActor(actor)
    
        # Clean up the measurement_actors list
        self.measurement_actors = [
            actor for actor in self.measurement_actors 
            if not (isinstance(actor, vtk.vtkFollower) and 
                    hasattr(actor.GetMapper().GetInputConnection(0, 0), 'GetProducer') and
                    isinstance(actor.GetMapper().GetInputConnection(0, 0).GetProducer(), vtk.vtkVectorText) and
                    actor.GetMapper().GetInputConnection(0, 0).GetProducer().GetText().startswith("Surface-"))
        ]
        self.measurement_actors = [
            actor for actor in self.measurement_actors 
            if not (isinstance(actor.GetMapper().GetInput(), vtk.vtkPolyData) and 
                    actor.GetProperty().GetOpacity() < 1.0)
        ]
        if hasattr(self, 'surface_info'):
            del self.surface_info

# =======================================================================================================================================
# Define the function for the cut the surface:
    def cut_surface(self):
        """Cut the selected surface from the point cloud and darken remaining surfaces"""
        if not hasattr(self, 'surface_info') or not self.surface_info:
            # self.output_list.addItem("No surfaces identified for cutting")
            return
        
        selected_idx = self.surface_combo.currentIndex()
        if selected_idx < 0 or selected_idx >= len(self.surface_info):
            # self.output_list.addItem("Please select a surface to cut first")
            return
        
        surface = self.surface_info[selected_idx]
        surface_points = surface['points']
        
        if len(surface_points) < 3:
            # self.output_list.addItem("Selected surface doesn't have enough points")
            return
        
        # Create a polygon path from surface points (projected to XY plane)
        polygon_path = matplotlib.path.Path([(p[0], p[1]) for p in surface_points])
        
        # Get all points from the point cloud
        points = np.asarray(self.point_cloud.points)
        colors = np.asarray(self.point_cloud.colors) if self.point_cloud.has_colors() else None
        
        # Find points inside the polygon
        inside = polygon_path.contains_points(points[:, :2])
        outside_points = points[~inside]
        
        if colors is not None:
            outside_colors = colors[~inside]
        
        # Create new point cloud with only points outside the polygon
        new_cloud = o3d.geometry.PointCloud()
        new_cloud.points = o3d.utility.Vector3dVector(outside_points)
        
        if colors is not None:
            new_cloud.colors = o3d.utility.Vector3dVector(outside_colors)
        
        # Replace the original point cloud
        self.point_cloud = new_cloud
        
        # Redraw the point cloud
        self.display_point_cloud()
        
        # Remove the cut surface from visualization
        self.remove_surface_visualization(selected_idx)
        
        # Darken remaining surfaces by increasing opacity
        for actor in self.measurement_actors:
            if isinstance(actor.GetMapper().GetInput(), vtk.vtkPolyData):
                if actor.GetProperty().GetOpacity() < 1.0:  # Our surfaces are transparent
                    actor.GetProperty().SetOpacity(0.6)  # Increase opacity
        
        # self.output_list.addItem(f"Cut {surface['label']} - removed {np.sum(inside)} points")
        self.cutting_mode = False
        self.cut_surface_button.setChecked(False)
        self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================
# Define the function remove the surface color:
    def remove_surface_visualization(self, surface_idx):
        """Remove a specific surface visualization after cutting"""
        if not hasattr(self, 'surface_info') or surface_idx >= len(self.surface_info):
            return
        
        # Remove the surface actor
        surface_count = 0
        actors_to_remove = []
        
        for actor in self.measurement_actors:
            if isinstance(actor.GetMapper().GetInput(), vtk.vtkPolyData):
                if actor.GetProperty().GetOpacity() < 1.0:  # Our surfaces are transparent
                    if surface_count == surface_idx:
                        actors_to_remove.append(actor)
                        # Also remove its label
                        label_actor = self.find_surface_label_actor(surface_idx)
                        if label_actor:
                            actors_to_remove.append(label_actor)
                        break
                    surface_count += 1
        
        # Remove the actors
        for actor in actors_to_remove:
            self.renderer.RemoveActor(actor)
            if actor in self.measurement_actors:
                self.measurement_actors.remove(actor)
        
        # Remove from surface info
        if surface_idx < len(self.surface_info):
            del self.surface_info[surface_idx]
        
        # Update the dropdown
        self.update_surface_combo()
        
        if not self.surface_info:
            self.surface_selection_group.setVisible(False)

# =======================================================================================================================================
    def find_surface_label_actor(self, surface_idx):
        """Find the label actor for a specific surface"""
        if not hasattr(self, 'surface_info') or surface_idx >= len(self.surface_info):
            return None
        
        target_label = self.surface_info[surface_idx]['label']
        
        for actor in self.measurement_actors:
            if isinstance(actor, vtk.vtkFollower):
                try:
                    text_source = actor.GetMapper().GetInputConnection(0, 0).GetProducer()
                    if isinstance(text_source, vtk.vtkVectorText):
                        if text_source.GetText() == target_label:
                            return actor
                except:
                    continue
        return None

# =======================================================================================================================================
# Define the function for the surfaces movement:
    def toggle_surface_movement(self, checked):
        """Toggle surface movement mode"""
        self.movement_mode = checked
        self.cutting_mode = False
        self.cut_surface_button.setChecked(False)
    
        if checked:
            # self.output_list.addItem("Surface movement mode activated")
            self.extract_surfaces_to_new_window()

# =======================================================================================================================================   
# Define the function for create new window for the visualization of extracted 3D polygon:
    def extract_surfaces_to_new_window(self):
        """Extract all identified surfaces to a new window for visualization"""
        if not hasattr(self, 'surface_info') or not self.surface_info:
            # self.output_list.addItem("No surfaces identified to extract")
            return

        # Calculate volume before creating the window
        all_points = list(self.measurement_points)
        if hasattr(self, 'digging_points_info') and self.digging_points_info:
            for dp in self.digging_points_info:
                all_points.append(dp['point'])

        points = np.array(all_points)
        volume_meters = self.calculate_volume(points)

        # Create a new window
        self.surface_window = QMainWindow()
        self.surface_window.setWindowTitle("Extracted Surfaces")
        self.surface_window.setGeometry(100, 100, 800, 600)

        # Create a main widget and layout for the window
        main_widget = QWidget()
        layout = QVBoxLayout()
        main_widget.setLayout(layout)

        # Create VTK widget for the new window
        vtk_widget = QVTKRenderWindowInteractor()
        renderer = vtkRenderer()
        vtk_widget.GetRenderWindow().AddRenderer(renderer)

        # Add VTK widget to layout
        layout.addWidget(vtk_widget)

        # Add measurement metrics controls at the top
        metrics_group = QGroupBox("Measurement Units")
        metrics_layout = QHBoxLayout()
        
        metrics_label = QLabel("Units:")
        metrics_combo = QComboBox()
        metrics_combo.addItems(["Meter", "Centimeter", "Millimeter"])
        
        # Set current selection to match main window
        if hasattr(self, 'metrics_combo'):
            metrics_combo.setCurrentText(self.metrics_combo.currentText())
        
        metrics_layout.addWidget(metrics_label)
        metrics_layout.addWidget(metrics_combo)
        metrics_group.setLayout(metrics_layout)
        layout.addWidget(metrics_group)

        # Add volume display label at the bottom
        volume_label = QLabel()
        volume_label.setAlignment(Qt.AlignCenter)
        volume_label.setStyleSheet("""
            QLabel {
                font-size: 16px;
                font-weight: bold;
                padding: 10px;
                background-color: #f0f0f0;
                border-top: 1px solid #ccc;
            }
        """)
        
        def update_volume_display():
            """Update the volume display with current units"""
            if volume_meters is not None:
                # Get current units from the combo box
                current_units = metrics_combo.currentText()
                
                if current_units == "Meter":
                    volume = volume_meters
                    volume_suffix = "cubic meters"
                elif current_units == "Centimeter":
                    volume = volume_meters * (100**3)  # m³ to cm³
                    volume_suffix = "cubic cm"
                elif current_units == "Millimeter":
                    volume = volume_meters * (1000**3)  # m³ to mm³
                    volume_suffix = "cubic mm"
                
                volume_label.setText(f"Volume: {volume:.3f} {volume_suffix}")
        
        # Initial volume display
        update_volume_display()
        
        # Connect combo box change to update display
        metrics_combo.currentTextChanged.connect(update_volume_display)
        
        layout.addWidget(volume_label)

        # Add each surface to the new window
        for i, surface in enumerate(self.surface_info):
            points = surface['points']
            color = surface['color']
            
            # Calculate centroid for label placement
            centroid = np.mean(points, axis=0)

            # Create a polygon from the points
            polygon = vtk.vtkPolygon()
            polygon.GetPointIds().SetNumberOfIds(len(points))
            for j, point in enumerate(points):
                polygon.GetPointIds().SetId(j, j)

            polygons = vtk.vtkCellArray()
            polygons.InsertNextCell(polygon)

            polygonPolyData = vtk.vtkPolyData()
            polygonPolyData.SetPoints(vtk.vtkPoints())
            for point in points:
                polygonPolyData.GetPoints().InsertNextPoint(point)
            polygonPolyData.SetPolys(polygons)

            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(polygonPolyData)

            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(self.colors.GetColor3d(color))
            actor.GetProperty().SetOpacity(0.6)
            actor.GetProperty().SetEdgeColor(self.colors.GetColor3d("White"))
            actor.GetProperty().SetEdgeVisibility(1)

            renderer.AddActor(actor)

            # Add label at centroid
            text = vtk.vtkVectorText()
            text.SetText(surface['label'])

            text_mapper = vtk.vtkPolyDataMapper()
            text_mapper.SetInputConnection(text.GetOutputPort())

            text_actor = vtk.vtkFollower()
            text_actor.SetMapper(text_mapper)
            text_actor.SetScale(0.1, 0.1, 0.1)
            text_actor.AddPosition(centroid[0], centroid[1], centroid[2])
            text_actor.GetProperty().SetColor(self.colors.GetColor3d("White"))

            renderer.AddActor(text_actor)
            text_actor.SetCamera(renderer.GetActiveCamera())

        # Add original polygon points as spheres
        for i, point in enumerate(self.measurement_points):
            sphere = vtkSphereSource()
            sphere.SetRadius(0.3)
            sphere.SetCenter(point[0], point[1], point[2])

            mapper = vtkPolyDataMapper()
            mapper.SetInputConnection(sphere.GetOutputPort())

            actor = vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(self.colors.GetColor3d("Red"))

            renderer.AddActor(actor)

            # Add label
            label = chr(65 + i)
            text = vtk.vtkVectorText()
            text.SetText(label)

            text_mapper = vtk.vtkPolyDataMapper()
            text_mapper.SetInputConnection(text.GetOutputPort())

            text_actor = vtk.vtkFollower()
            text_actor.SetMapper(text_mapper)
            text_actor.SetScale(0.1, 0.1, 0.1)
            text_actor.AddPosition(point[0] + 0.15, point[1] + 0.15, point[2])
            text_actor.GetProperty().SetColor(self.colors.GetColor3d("White"))

            renderer.AddActor(text_actor)
            text_actor.SetCamera(renderer.GetActiveCamera())

        renderer.SetBackground(1.0, 1.0, 1.0)
        renderer.ResetCamera()

        # Set the main widget as central widget
        self.surface_window.setCentralWidget(main_widget)

        # Initialize and render carefully
        vtk_widget.Initialize()
        QApplication.processEvents()  # Ensure GUI is ready
        vtk_widget.GetRenderWindow().Render()

        self.surface_window.show()
        vtk_widget.Initialize()
        vtk_widget.GetRenderWindow().Render()
        QApplication.processEvents()

        # self.output_list.addItem("All surfaces extracted to new window")
        if volume_meters is not None:
            # Display volume in main window's current units
            units_suffix, conversion_factor = self.get_current_units()
            if units_suffix == "cm":
                volume = volume_meters * (100**3)
                volume_suffix = "cubic cm"
            elif units_suffix == "mm":
                volume = volume_meters * (1000**3)
                volume_suffix = "cubic mm"
            else:
                volume = volume_meters
                volume_suffix = "cubic meters"
                
            self.output_list.addItem(f"Volume displayed in extracted window: {volume:.3f} {volume_suffix}")

# =======================================================================================================================================      
# Define the function for the calculate the volume of selected 3D Geometry:
    def calculate_selected_volume(self):
        """Calculate volume of selected points or entire geometry"""
        if not hasattr(self, 'measurement_points') or len(self.measurement_points) < 4:
            # self.output_list.addItem("Need at least 4 points to calculate volume")
            return

        # Combine polygon points and digging points if available
        all_points = list(self.measurement_points)

        if hasattr(self, 'digging_points_info') and self.digging_points_info:
            for dp in self.digging_points_info:
                all_points.append(dp['point'])

        points = np.array(all_points)
        volume_meters = self.calculate_volume(points)

        if volume_meters is not None:
            # Convert to current units
            units_suffix, conversion_factor = self.get_current_units()
            
            # For volume, we need to cube the conversion factor
            if units_suffix == "cm":
                volume = volume_meters * (100**3)  # m³ to cm³
                volume_suffix = "cubic cm"
            elif units_suffix == "mm":
                volume = volume_meters * (1000**3)  # m³ to mm³
                volume_suffix = "cubic mm"
            else:
                volume = volume_meters
                volume_suffix = "cubic meter"
                
            self.output_list.addItem(f"Calculated volume: {volume:.3f} {volume_suffix}")
            self.volume_check.setChecked(True)

# =======================================================================================================================================
# define the function for cropped the selected area by rectangle::
    def crop_selected_area(self):
        """Crop the point cloud based on the selected rectangle/polygon"""
        if not hasattr(self, 'measurement_points') or len(self.measurement_points) < 3:
            # self.output_list.addItem("Please select an area first (rectangle or polygon)")
            return
        
        if not self.point_cloud:
            self.output_list.addItem("No point cloud data to crop")
            return
        
        try:
            # Get all points from the point cloud
            points = np.asarray(self.point_cloud.points)
            colors = np.asarray(self.point_cloud.colors) if self.point_cloud.has_colors() else None
            
            # Create a polygon path from measurement points (projected to XY plane)
            polygon_path = matplotlib.path.Path([(p[0], p[1]) for p in self.measurement_points])
            
            # Find points inside the polygon
            inside = polygon_path.contains_points(points[:, :2])
            
            # Create new point cloud with only points inside the polygon
            self.cropped_cloud = o3d.geometry.PointCloud()
            self.cropped_cloud.points = o3d.utility.Vector3dVector(points[inside])
            
            if colors is not None:
                self.cropped_cloud.colors = o3d.utility.Vector3dVector(colors[inside])
            
            # Display the cropped cloud
            self.display_cropped_cloud()
            
            # self.output_list.addItem(f"Cropped area - kept {np.sum(inside)} points")
            self.save_crop_button.setEnabled(True)
            
        except Exception as e:
            self.output_list.addItem(f"Error cropping area: {str(e)}")

# =======================================================================================================================================
# Define the function fro the display the cropped point cloud data in new window:
    def display_cropped_cloud(self):
        """Display the cropped point cloud in a new window"""
        if not self.cropped_cloud:
            return
        
        # Create a new window
        self.crop_window = QMainWindow()
        self.crop_window.setWindowTitle("Cropped Point Cloud")
        self.crop_window.setGeometry(100, 100, 800, 600)
        
        # Create VTK widget for the new window
        vtk_widget = QVTKRenderWindowInteractor(self.crop_window)
        renderer = vtkRenderer()
        vtk_widget.GetRenderWindow().AddRenderer(renderer)
        
        # Convert Open3D point cloud to VTK format
        points = np.asarray(self.cropped_cloud.points)
        
        # Create VTK points
        vtk_points = vtk.vtkPoints()
        for point in points:
            vtk_points.InsertNextPoint(point[0], point[1], point[2])
        
        # Create VTK polydata
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(vtk_points)
        
        # Create vertex cells
        vertices = vtk.vtkCellArray()
        for i in range(points.shape[0]):
            vertices.InsertNextCell(1)
            vertices.InsertCellPoint(i)
        polydata.SetVerts(vertices)
        
        # Add color information if available
        if self.cropped_cloud.has_colors():
            colors = np.asarray(self.cropped_cloud.colors) * 255
            vtk_colors = vtk.vtkUnsignedCharArray()
            vtk_colors.SetNumberOfComponents(3)
            vtk_colors.SetName("Colors")
            
            for color in colors:
                vtk_colors.InsertNextTuple3(color[0], color[1], color[2])
            
            polydata.GetPointData().SetScalars(vtk_colors)
        
        # Create mapper and actor
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(polydata)
        
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetPointSize(2)
        
        if not self.cropped_cloud.has_colors():
            actor.GetProperty().SetColor(self.colors.GetColor3d("Black"))
        
        renderer.AddActor(actor)
        renderer.ResetCamera()
        
        # Add save button at the bottom
        save_button = QPushButton("Save This View")
        save_button.clicked.connect(lambda: self.save_cropped_data(vtk_widget))
        
        # Create layout
        layout = QVBoxLayout()
        layout.addWidget(vtk_widget)
        layout.addWidget(save_button)
        
        container = QWidget()
        container.setLayout(layout)
        self.crop_window.setCentralWidget(container)
        
        # Initialize and show
        vtk_widget.Initialize()
        self.crop_window.show()

# =======================================================================================================================================
# Define the function for the Save Cropped Data:
    def save_cropped_data(self, vtk_widget=None):
        """Save the cropped point cloud to a file"""
        if not self.cropped_cloud:
            self.output_list.addItem("No cropped data to save")
            return
        
        try:
            # Get file path from user
            file_dialog = QFileDialog()
            file_path, _ = file_dialog.getSaveFileName(
                self, "Save Cropped Point Cloud", "", 
                "Point Cloud Files (*.ply *.pcd);;All Files (*)")
            
            if not file_path:
                return
            
            # Add extension if not provided
            if not file_path.lower().endswith(('.ply', '.pcd')):
                file_path += '.ply'
            
            # Save the file
            o3d.io.write_point_cloud(file_path, self.cropped_cloud)
            self.output_list.addItem(f"Cropped point cloud saved to {file_path}")
            
            # If saving from the crop window, close it
            if vtk_widget:
                self.crop_window.close()
                
        except Exception as e:
            self.output_list.addItem(f"Error saving cropped data: {str(e)}")

# =======================================================================================================================================    
# Volume calculation of Extracted 3D Polygon Geometry by using the Delaunay Triangulation
    def calculate_volume(self, points):
        """
        Calculates the volume of an irregular 3D shape using Delaunay triangulation.
    
        Args:
            points: A numpy array of shape (n, 3) representing the vertices of the shape.
        
        Returns:
            The volume of the shape.
        """
        try:
            from scipy.spatial import Delaunay
            import numpy as np
        
            tri = Delaunay(points)
            tetrahedra = points[tri.simplices]
        
            total_volume = 0
            for tetrahedron in tetrahedra:
                x0, y0, z0 = tetrahedron[0]
                x1, y1, z1 = tetrahedron[1]
                x2, y2, z2 = tetrahedron[2]
                x3, y3, z3 = tetrahedron[3]
            
                volume = (1/6) * abs(np.linalg.det(np.array([[x1-x0, y1-y0, z1-z0],
                                                         [x2-x0, y2-y0, z2-z0],
                                                         [x3-x0, y3-y0, z3-z0]])))
                total_volume += volume
            return total_volume
        
        except ImportError:
            self.output_list.addItem("Error: scipy package required for volume calculation")
            return None
        except Exception as e:
            self.output_list.addItem(f"Error calculating volume: {str(e)}")
            return None
        
# =======================================================================================================================================
# Define the function for the highlight the surface of 3D Polygon:
    def highlight_selected_surface(self):
        """Highlight the selected surface by changing its color to green while keeping others original"""
        selected_idx = self.surface_combo.currentIndex()
        
        if 0 <= selected_idx < len(getattr(self, 'surface_info', [])):
            # First reset all surface colors to their original colors
            for i, surface in enumerate(self.surface_info):
                for actor in self.measurement_actors:
                    if isinstance(actor.GetMapper().GetInput(), vtk.vtkPolyData):
                        if actor.GetProperty().GetOpacity() < 1.0:  # Our surfaces are transparent
                            # Find matching surface by checking label position
                            text_actor = self.find_surface_label_actor(i)
                            if text_actor:
                                label_pos = text_actor.GetPosition()
                                surface_center = np.mean([actor.GetMapper().GetInput().GetPoint(j) 
                                                    for j in range(actor.GetMapper().GetInput().GetNumberOfPoints())], 
                                                    axis=0)
                                if np.linalg.norm(np.array(label_pos) - surface_center) < 0.5:
                                    # Reset to original color
                                    actor.GetProperty().SetColor(self.colors.GetColor3d(surface['color']))
                                    actor.GetProperty().SetOpacity(0.3)  # Original opacity
            
            # Now highlight only the selected surface in green
            surface = self.surface_info[selected_idx]
            surface_label = surface['label']
            
            # Find and change the color of the selected surface actor
            for actor in self.measurement_actors:
                if isinstance(actor.GetMapper().GetInput(), vtk.vtkPolyData):
                    if actor.GetProperty().GetOpacity() < 1.0:  # Our surfaces are transparent
                        # Check if this is the actor for our selected surface
                        text_actor = self.find_surface_label_actor(selected_idx)
                        if text_actor:
                            label_pos = text_actor.GetPosition()
                            surface_center = np.mean([actor.GetMapper().GetInput().GetPoint(i) 
                                                for i in range(actor.GetMapper().GetInput().GetNumberOfPoints())], 
                                                axis=0)
                            if np.linalg.norm(np.array(label_pos) - surface_center) < 0.5:
                                actor.GetProperty().SetColor(self.colors.GetColor3d("Green"))
                                actor.GetProperty().SetOpacity(0.6)  # Make highlighted surface more visible
                                break
            
            # self.output_list.addItem(f"Highlighted {surface_label} in green")
            self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================
# Define function to process pillar dimensions
    def process_pillar_dimension(self):
        if (
            hasattr(self, 'round_pillar_volume_meters') and
            hasattr(self, 'round_pillar_outer_surface_meters') and
            hasattr(self, 'polygon_volume_meters') and
            hasattr(self, 'polygon_outer_surface_meters') and
            self.round_pillar_volume_meters > 0 and
            self.round_pillar_outer_surface_meters > 0 and
            self.polygon_volume_meters > 0 and
            self.polygon_outer_surface_meters > 0
            ):

            try:
                round_pillar_points = self.round_pillar_points
                polygon_points = self.polygon_points

                # --- COMBINED centroid of round + polygon ---
                all_points = np.vstack((round_pillar_points, polygon_points))
                centroid = np.mean(all_points, axis=0)

                # Convert to current units
                units_suffix, conversion_factor = self.get_current_units()

                total_pillar_volume = (self.round_pillar_volume_meters + self.polygon_volume_meters)
                self.total_pillar_volume_meters = total_pillar_volume

                total_pillar_outer_surface = (self.round_pillar_outer_surface_meters + self.polygon_outer_surface_meters)
                self.total_pillar_outer_surface_meters = total_pillar_outer_surface

                # Convert measurements to current units
                if units_suffix == "cm":
                    volume = total_pillar_volume * 1000000  # m³ to cm³
                    outer_surface = total_pillar_outer_surface * 10000  # m² to cm²
                    area_suffix = "square cm"
                    volume_suffix = "cubic cm"
                elif units_suffix == "mm":
                    volume = total_pillar_volume * 1000000000  # m³ to mm³
                    outer_surface = total_pillar_outer_surface * 1000000  # m² to mm²
                    area_suffix = "square mm"
                    volume_suffix = "cubic mm"
                else:
                    volume = total_pillar_volume
                    outer_surface = total_pillar_outer_surface
                    area_suffix = "square meter"
                    volume_suffix = "cubic meter"

                # Add labels for volume and outer surface
                self.add_text_label(centroid + np.array([0, 2, 1]), f"Total Pillar Volume = {volume:.2f} {volume_suffix}", "Blue")
                self.add_text_label(centroid + np.array([0, 2, 2]), f"Total Pillar Outer Surface = {outer_surface:.2f} {area_suffix}", "Blue")

                self.output_list.addItem(f"Total Pillar Volume: {volume:.2f} {volume_suffix}")
                self.output_list.addItem(f"Total Pillar Outer Surface: {outer_surface:.2f} {area_suffix}")

            except Exception as e:
                QMessageBox.critical(self, "Error processing pillar dimensions", str(e))

        else:
            # Show warning when attributes missing or invalid
            QMessageBox.warning(
                self,
                "Missing Data",
                "First calculate polygon and round pillar polygon volume."
            )

# =======================================================================================================================================
# Define the function for the create a baseline at height::
    def baseline(self):
        """Handle baseline creation - get height from user and draw baseline plane"""
        # Get baseline height from user
        for actor in self.baseline_actors:
            self.renderer.RemoveActor(actor)
        self.baseline_actors = []

        height, ok = QInputDialog.getDouble(
            self,  # parent
            "Baseline Height",  # title
            "Enter baseline height above minimum Z (meters):",  # label
            value=1.0,  # default value
            min=0.1,  # minimum value (changed from minValue to min)
            max=100.0,  # maximum value (changed from maxValue to max)
            decimals=2  # decimal places
        )
        
        if not ok or not self.point_cloud:
            return
        
        try:
            # Calculate minimum Z of point cloud
            points = np.asarray(self.point_cloud.points)
            if len(points) == 0:
                self.output_list.addItem("No points in cloud to calculate baseline")
                return
                
            min_z = np.min(points[:, 2])
            baseline_z = min_z + height
            
            # Calculate bounding box dimensions
            min_pt = np.min(points, axis=0)
            max_pt = np.max(points, axis=0)
            
            # Create baseline plane (blue rectangle covering XY extent)
            plane_points = [
                [min_pt[0], min_pt[1], baseline_z],  # Bottom-left
                [max_pt[0], min_pt[1], baseline_z],  # Bottom-right
                [max_pt[0], max_pt[1], baseline_z],  # Top-right
                [min_pt[0], max_pt[1], baseline_z]   # Top-left
            ]
            
            # Create polygon from points
            polygon = vtk.vtkPolygon()
            polygon.GetPointIds().SetNumberOfIds(4)
            for i in range(4):
                polygon.GetPointIds().SetId(i, i)
            
            polygons = vtk.vtkCellArray()
            polygons.InsertNextCell(polygon)
            
            polygonPolyData = vtk.vtkPolyData()
            polygonPolyData.SetPoints(vtk.vtkPoints())
            for point in plane_points:
                polygonPolyData.GetPoints().InsertNextPoint(point)
            polygonPolyData.SetPolys(polygons)
            
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(polygonPolyData)
            
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(self.colors.GetColor3d("Blue"))
            actor.GetProperty().SetOpacity(0.5)  # Semi-transparent
            actor.GetProperty().SetEdgeVisibility(1)
            actor.GetProperty().SetEdgeColor(self.colors.GetColor3d("White"))
            
            self.renderer.AddActor(actor)
            self.baseline_actors.append(actor)
            
            # Add label at center of baseline
            center = [
                (min_pt[0] + max_pt[0]) / 2,
                (min_pt[1] + max_pt[1]) / 2,
                baseline_z
            ]
            self.add_text_label(center, f"Baseline: {height:.2f}m above min Z", "White")
            
            self.output_list.addItem(f"Baseline created at Z = {baseline_z:.2f} (min Z: {min_z:.2f} + height: {height:.2f})")
            self.output_list.addItem(f"Baseline dimensions: X = {(max_pt[0]-min_pt[0]):.2f}m, Y = {(max_pt[1]-min_pt[1]):.2f}m")
            
            self.vtk_widget.GetRenderWindow().Render()
            
        except Exception as e:
            self.output_list.addItem(f"Error creating baseline: {str(e)}")

# =======================================================================================================================================
    def find_intersection_with_baseline(self, point):
        """Find the intersection of a vertical line from the given point to the baseline"""
        if not hasattr(self, 'baseline_actors') or not self.baseline_actors:
            return None
        
        # Get baseline Z value from the first baseline actor
        baseline_actor = self.baseline_actors[0]
        bounds = baseline_actor.GetBounds()
        baseline_z = bounds[4]  # Minimum Z of the baseline plane
        
        # Return point directly below input point but on baseline
        return np.array([point[0], point[1], baseline_z])

# =======================================================================================================================================   
# Define the function for the calculate the inclination angle::
    def calculate_inclination_angle(self):
        """Calculate the inclination angle of the selected line"""
        if not hasattr(self, 'measurement_points') or len(self.measurement_points) < 2:
            self.output_list.addItem("Please create a line measurement first")
            return
        
        # Get the two points of the line
        p1 = self.measurement_points[0]
        p2 = self.measurement_points[1]
        
        # Calculate the vector between points
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dz = p2[2] - p1[2]
        
        # Calculate horizontal distance (projection on XY plane)
        horizontal_distance_meters = np.sqrt(dx**2 + dy**2)

        units_suffix, conversion_factor = self.get_current_units()
        horizontal_distance = horizontal_distance_meters * conversion_factor
        
        # Calculate vertical distance (Z component)
        vertical_distance_meters = abs(dz)

        units_suffix, conversion_factor = self.get_current_units()
        vertical_distance = vertical_distance_meters * conversion_factor
        
        # Calculate inclination angle in degrees
        if horizontal_distance > 0:
            inclination_angle = np.degrees(np.arctan(vertical_distance / horizontal_distance))
        else:
            # Vertical line case (90 degrees)
            inclination_angle = 90.0
        
        # Determine line type
        if abs(dz) > 0.1:  # Threshold for considering it a vertical line
            line_type = "Vertical"
        else:
            line_type = "Horizontal"
        
        # Output results
        self.output_list.addItem(f"Inclination Angle Calculation ({line_type} Line):")
        # self.output_list.addItem(f"Point 1: X={p1[0]:.3f}, Y={p1[1]:.3f}, Z={p1[2]:.3f}")
        # self.output_list.addItem(f"Point 2: X={p2[0]:.3f}, Y={p2[1]:.3f}, Z={p2[2]:.3f}")
        self.output_list.addItem(f"Horizontal Distance: {horizontal_distance:.3f} {units_suffix}")
        self.output_list.addItem(f"Vertical Distance: {vertical_distance:.2f} {units_suffix}")
        self.output_list.addItem(f"Inclination Angle: {inclination_angle:.2f}°")
        
        # Visualize the angle with an arc if it's a vertical line
        if line_type == "Vertical":
            self.visualize_inclination_angle(p1, p2, inclination_angle)

# ======================================================================================================================================= 
# Define the function for the Inclination Angle::
    def visualize_inclination_angle(self, p1, p2, angle):
        """Visualize the inclination angle with an arc"""
        # Remove any existing inclination visualization
        for actor in self.measurement_actors:
            if hasattr(actor, 'is_inclination_visualization'):
                self.renderer.RemoveActor(actor)
                self.measurement_actors.remove(actor)
        
        # Calculate midpoint for arc placement
        midpoint = (p1 + p2) / 2
        
        # Create arc source
        arc = vtk.vtkArcSource()
        arc.SetPoint1(p1[0], p1[1], p1[2])
        arc.SetPoint2(p2[0], p2[1], p2[2])
        arc.SetCenter(midpoint[0], midpoint[1], midpoint[2])
        arc.SetResolution(30)
        
        # Create mapper and actor
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(arc.GetOutputPort())
        
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(self.colors.GetColor3d("Yellow"))
        actor.GetProperty().SetLineWidth(2)
        actor.is_inclination_visualization = True
        
        # Add to renderer
        self.renderer.AddActor(actor)
        self.measurement_actors.append(actor)
        
        # Add angle label
        label_pos = p2 + np.array([0.2, 0.2, 0])  # Offset slightly
        self.add_text_label(label_pos, f"{angle:.1f}°", "Yellow")
        
        self.vtk_widget.GetRenderWindow().Render()

# =======================================================================================================================================
# Define the function for the handle the Presized button action:
    def handle_presized_button(self):
        """Handle the Presized button click for round pillar measurement."""
        if (hasattr(self, 'current_measurement') and 
            self.current_measurement == 'vertical_line' and 
            hasattr(self, 'measurement_points') and 
            len(self.measurement_points) >= 2 and
            hasattr(self, 'round_pillar_surface_area_meters') and
            self.round_pillar_surface_area_meters > 0):
            
            try:
                # Get points A and B from the original measurement
                point_a = self.measurement_points[0]
                point_b = self.measurement_points[1]
                
                # Create point C at the same height as B but directly above A
                point_c = np.array([point_a[0], point_a[1], point_b[2]])
                self.plotting_active = False

                height_meters= self.create_presized_vertical_line()

                # Convert to current units
                units_suffix, conversion_factor = self.get_current_units()
                height_AC = height_meters * conversion_factor

                # Calculate volume and surface area using the round pillar polygon data
                surface_area_meters = self.round_pillar_surface_area_meters
                perimeter_meters = self.round_pillar_perimeter_meters
                volume_meters = surface_area_meters * height_meters
                outer_surface_meters = perimeter_meters * height_meters

                self.round_pillar_volume_meters = volume_meters
                self.round_pillar_outer_surface_meters = outer_surface_meters

                # --- UPDATE LAST STORED ENTRY ---
                if hasattr(self, "all_presized_vertical_lines") and self.all_presized_vertical_lines:
                    self.all_presized_vertical_lines[-1]["volume"] = volume_meters
                    self.all_presized_vertical_lines[-1]["outer_surface"] = outer_surface_meters

                # Convert measurements to current units
                if units_suffix == "cm":
                    surface_area = surface_area_meters * 10000  # m² to cm²
                    volume = volume_meters * 1000000  # m³ to cm³
                    outer_surface = outer_surface_meters * 10000  # m² to cm²
                    perimeter = perimeter_meters * 100
                    area_suffix = "square cm"
                    volume_suffix = "cubic cm"
                    perimeter_suffix = "centi meter"
                elif units_suffix == "mm":
                    surface_area = surface_area_meters * 1000000  # m² to mm²
                    volume = volume_meters * 1000000000  # m³ to mm³
                    outer_surface = outer_surface_meters * 1000000  # m² to mm²
                    perimeter = perimeter_meters * 1000
                    area_suffix = "square mm"
                    volume_suffix = "cubic mm"
                    perimeter_suffix = "mili meter"
                else:
                    surface_area = surface_area_meters
                    volume = volume_meters
                    outer_surface = outer_surface_meters
                    perimeter = perimeter_meters
                    area_suffix = "square meter"
                    volume_suffix = "cubic meter"
                    perimeter_suffix = "meter"

                # Calculate centroid for label placement
                if hasattr(self, 'measurement_points') and len(self.measurement_points) >= 3:
                    polygon_points = np.array(self.measurement_points)
                    centroid = np.mean(polygon_points, axis=0)
                else:
                    centroid = (point_a + point_c) / 2

                # Add labels for volume and outer surface
                self.add_text_label(centroid + np.array([1, 2, 2]), 
                                    f"Round Pillar Area = {surface_area:.2f} {area_suffix}, "
                                    f"Round Pillar Volume = {volume:.2f} {volume_suffix}",
                                    "Orange")
                self.add_text_label(centroid + np.array([1, 2, 1]),
                                    f"Round Pillar Outer Surface = {outer_surface:.2f} {area_suffix}, "
                                    f"Round Pillar Perimeter = {perimeter:.2f} {perimeter_suffix}",
                                    "Orange")

                # Output results
                self.output_list.addItem(f"Surface Area of Round Pillar Polygon = {surface_area:.2f} {area_suffix}")
                self.output_list.addItem(f"Volume of Round Pillar Polygon = {volume:.2f} {volume_suffix}")
                self.output_list.addItem(f"Outer Surface Area of Round Pillar Polygon = {outer_surface:.2f} {area_suffix}")

                # Store measurement data
                self.measurement_widget.measurement_data.update({
                    'presized_height': height_AC,
                    'presized_volume': volume,
                    'presized_outer_surface': outer_surface
                })
                self.measurement_widget.update()
            
            except Exception as e:
                self.output_list.addItem(f"")
            
            self.vtk_widget.GetRenderWindow().Render()
            return
        
        if (hasattr(self, 'current_measurement') and 
            self.current_measurement == 'horizontal_line' and 
            hasattr(self, 'measurement_points') and 
            len(self.measurement_points) >= 2 and
            hasattr(self, 'polygon_area_meters') and
            self.polygon_area_meters > 0):

            try:
                # Get points A and B from the original measurement
                point_p = self.measurement_points[0]
                point_q = self.measurement_points[1]

                # Create point R: same X, Y as Q but Z as P
                point_r = np.array([point_q[0], point_q[1], point_p[2]])
                
                distance_meters = self.create_presized_horizontal_line()

                # Convert to current units
                units_suffix, conversion_factor = self.get_current_units()
                distance_PR = distance_meters * conversion_factor

                # Calculate volume and surface area using the round pillar polygon data
                surface_area_meters = self.polygon_area_meters
                perimeter_meters = self.polygon_perimeter_meters
                volume_meters = surface_area_meters * distance_meters
                outer_surface_meters = perimeter_meters * distance_meters

                self.polygon_volume_meters = volume_meters
                self.polygon_outer_surface_meters = outer_surface_meters

                if hasattr(self, "all_presized_horizontal_lines") and self.all_presized_horizontal_lines:
                    self.all_presized_horizontal_lines[-1]["volume"] = volume_meters
                    self.all_presized_horizontal_lines[-1]["outer_surface"] = outer_surface_meters

                # Convert measurements to current units
                if units_suffix == "cm":
                    surface_area = surface_area_meters * 10000  # m² to cm²
                    volume = volume_meters * 1000000  # m³ to cm³
                    outer_surface = outer_surface_meters * 10000  # m² to cm²
                    perimeter = perimeter_meters * 100
                    area_suffix = "square cm"
                    volume_suffix = "cubic cm"
                    perimeter_suffix = "centi meter"
                elif units_suffix == "mm":
                    surface_area = surface_area_meters * 1000000  # m² to mm²
                    volume = volume_meters * 1000000000  # m³ to mm³
                    outer_surface = outer_surface_meters * 1000000  # m² to mm²
                    perimeter = perimeter_meters * 1000
                    area_suffix = "square mm"
                    volume_suffix = "cubic mm"
                    perimeter_suffix = "mili meter"
                else:
                    surface_area = surface_area_meters
                    volume = volume_meters
                    outer_surface = outer_surface_meters
                    perimeter = perimeter_meters
                    area_suffix = "square meter"
                    volume_suffix = "cubic meter"
                    perimeter_suffix = "meter"
                
                # Calculate centroid for label placement
                if hasattr(self, 'measurement_points') and len(self.measurement_points) >= 3:
                    polygon_points = np.array(self.measurement_points)
                    centroid = np.mean(polygon_points, axis=0)
                else:
                    centroid = (point_p + point_r) / 2
                
                # Add labels for volume and outer surface
                self.add_text_label(centroid + np.array([0, 3, 2]), f"Polygon Volume = {volume:.2f} {volume_suffix}, "f"Polygon Outer Surface = {outer_surface:.2f} {area_suffix}", "Green")
                self.add_text_label(centroid + np.array([0, 3, 1]), f"Polygon Area = {surface_area:.2f} {area_suffix}, "f"Polygon Perimeter = {perimeter:.2f} {perimeter_suffix}", "Green")

                # Output results
                self.output_list.addItem(f"Surface Area of Polygon = {surface_area:.2f} {area_suffix}")
                self.output_list.addItem(f"Volume of Polygon = {volume:.2f} {volume_suffix}")
                self.output_list.addItem(f"Outer Surface Area of Polygon= {outer_surface:.2f} {area_suffix}")

                # Store measurement data
                self.measurement_widget.measurement_data.update({
                    'presized_horizontal_distance': distance_PR,
                    'presized_volume': volume,
                    'presized_outer_surface': outer_surface
                })
                self.measurement_widget.update()
            
            except Exception as e:
                self.output_list.addItem(f"")
            
            self.vtk_widget.GetRenderWindow().Render()
            return
        
        if (hasattr(self, 'current_measurement') and self.current_measurement == 'vertical_line' and \
            hasattr(self, 'measurement_points') and len(self.measurement_points) >= 2):
            # Vertical line case
            try:
                self.create_presized_vertical_line()
                # self.output_list.addItem("Presized vertical line created")
                
            except Exception as e:
                self.output_list.addItem(f"")

        if (hasattr(self, 'current_measurement') and self.current_measurement == 'horizontal_line' and \
            hasattr(self, 'measurement_points') and len(self.measurement_points) >= 2):
            # Horizontal line case
            try:
                self.create_presized_horizontal_line()                
                # self.output_list.addItem("Presized horizontal line created")
                
            except Exception as e:
                self.output_list.addItem(f"")

        """Handle the Presized button click - create accurate vertical measurement"""
        # Check if we have a measurement line with vertical
        if (hasattr(self, 'measurement_line_points') and len(self.measurement_line_points) >= 3 and
                hasattr(self, 'measurement_points') and len(self.measurement_points) >= 2):
                try:
                    M, N, O = self.measurement_line_points[:3]
                    A, B = self.measurement_points[:2]
                    
                    # Create point C at the same height as B but directly above A
                    point_c = np.array([A[0], A[1], B[2]])
                    
                    # Calculate the distance between A and C (vertical height)
                    distance_meters = np.linalg.norm(point_c - A)
                    
                    # Convert to current units
                    units_suffix, conversion_factor = self.get_current_units()
                    distance = distance_meters * conversion_factor
                    
                    # Change original AB line color to LightGrey
                    if hasattr(self, 'main_line_actor') and self.main_line_actor is not None:
                        self.main_line_actor.GetProperty().SetColor(self.colors.GetColor3d("LightGrey"))
                    
                    # Change point B sphere and label to LightGrey
                    if hasattr(self, 'point_b_actor') and self.point_b_actor is not None:
                        self.point_b_actor.GetProperty().SetColor(self.colors.GetColor3d("LightGrey"))
                    
                    # Change distance label to LightGrey
                    if hasattr(self, 'distance_label_actor') and self.distance_label_actor is not None:
                        self.distance_label_actor.GetProperty().SetColor(self.colors.GetColor3d("LightGrey"))
                    
                    # Draw the new presized vertical line from A to C in red
                    self.presized_line_actor = self.add_line_between_points(A, point_c, "Red", f"AC={distance:.2f}{units_suffix}")
                    
                    # Add point C marker with label
                    self.point_c_actor = self.add_sphere_marker(point_c, "C", color="Red")
                    
                    # Recalculate volume with presized height
                    MN_meters = np.linalg.norm(N - M)
                    NO_meters = np.linalg.norm(O - N)
                    surface_area_meters = MN_meters * NO_meters
                    volume_meters = surface_area_meters * distance_meters
                    perimeter_meters = 2 * (MN_meters + NO_meters)
                    outer_surface_meters = perimeter_meters * distance_meters
                    
                    # Convert measurements
                    if units_suffix == "cm":
                        surface_area = surface_area_meters * 10000
                        volume = volume_meters * 1000000
                        outer_surface = outer_surface_meters * 10000
                        area_suffix = "square cm"
                        volume_suffix = "cubic cm"
                    elif units_suffix == "mm":
                        surface_area = surface_area_meters * 1000000
                        volume = volume_meters * 1000000000
                        outer_surface = outer_surface_meters * 1000000
                        area_suffix = "square mm"
                        volume_suffix = "cubic mm"
                    else:
                        surface_area = surface_area_meters
                        volume = volume_meters
                        outer_surface = outer_surface_meters
                        area_suffix = "square meter"
                        volume_suffix = "cubic meter"
                    
                    # Update measurement labels
                    centroid = np.mean([M, N, O], axis=0)
                    self.add_text_label(centroid + np.array([0, 0, 1]), 
                                    f"Presized Volume={volume:.2f}{volume_suffix}", "Green")
                    #self.add_text_label(centroid + np.array([0, 0, 2]), f"Presized Outer Surface={outer_surface:.2f}{area_suffix}", "Green")
                    
                    # Output results
                    self.output_list.addItem("Presized Measurement Results:")
                    self.output_list.addItem(f"Presized Height AC = {distance:.3f} {units_suffix}")
                    self.output_list.addItem(f"Presized Volume = {volume:.3f} {volume_suffix}")
                    self.output_list.addItem(f"Presized Outer Surface Area = {outer_surface:.3f} {area_suffix}")
                    
                    # Update measurement data
                    self.measurement_widget.measurement_data.update({
                        'presized_lengths': {
                            'AC': distance
                        },
                        'presized_volume': volume,
                        'presized_outer_surface': outer_surface
                    })
                    self.measurement_widget.update()
                    
                except Exception as e:
                    self.output_list.addItem(f"Error in presizing measurement: {str(e)}")
                self.vtk_widget.GetRenderWindow().Render()
                return

# =======================================================================================================================================      
# define the function for the create a presized horizontal line::
    def create_presized_horizontal_line(self):
        """Create a new straight horizontal line from point P to point R with offset points in XY, XZ, YZ planes"""
        if not hasattr(self, 'measurement_points') or len(self.measurement_points) < 2:
            return
        
        try:
            # Get points P and Q from the original measurement
            point_p = self.measurement_points[0]
            point_q = self.measurement_points[1]

            # Create point R: same X, Y as Q but Z as P
            point_r = np.array([point_q[0], point_q[1], point_p[2]])

            # Calculate 3D distance between P and R
            distance_meters = np.linalg.norm(point_r - point_p)
            self.plotting_active = False

            # Store points & length for saving later
            self.presized_horizontal_points = [point_p.tolist(), point_r.tolist()]  # [P, R]
            self.presized_horizontal_length_meters = distance_meters

            # Store for multiple saving
            if not hasattr(self, "all_presized_horizontal_lines"):
                self.all_presized_horizontal_lines = []

            self.all_presized_horizontal_lines.append({
                "points": {"P": point_p.tolist(), "R": point_r.tolist()},
                "length_m": distance_meters
            })

            # Convert distance to current units
            units_suffix, conversion_factor = self.get_current_units()
            distance = distance_meters * conversion_factor

            # 1. Change horizontal line color from Red to LightGrey
            if self.horizontal_line_actor is not None:
                self.horizontal_line_actor.GetProperty().SetColor(self.colors.GetColor3d("LightGrey"))

            # 2. Change point Q sphere color to LightGrey
            if hasattr(self, 'point_q_actor') and self.point_q_actor is not None:
                self.point_q_actor.GetProperty().SetColor(self.colors.GetColor3d("LightGrey"))
        
            # 3. Change point Q label color to LightGrey
            for actor in self.measurement_actors:
                if isinstance(actor, vtk.vtkFollower):
                    try:
                        text_source = actor.GetMapper().GetInputConnection(0, 0).GetProducer()
                        if isinstance(text_source, vtk.vtkVectorText) and text_source.GetText() == "Q":
                            actor.GetProperty().SetColor(self.colors.GetColor3d("LightGrey"))
                            break
                    except:
                        continue

            # 4. Change distance label color to LightGrey and reposition it above the original PQ line
            if hasattr(self, 'horizontal_distance_label_actor') and self.horizontal_distance_label_actor is not None:
                self.horizontal_distance_label_actor.GetProperty().SetColor(self.colors.GetColor3d("LightGrey"))

            # Draw the new straight horizontal line from P to R in red
            self.presized_horizontal_line_actor = self.add_line_between_points(point_p, point_r, "Red", f"PR={distance:.2f}{units_suffix}")

            # Add point R marker with label
            self.point_r_actor = self.add_sphere_marker(point_r, "R", color="Red")

            self.output_list.addItem(f"Presized Horizontal Distance PR: {distance:.2f} {units_suffix}")

            # Update measurement widget
            self.measurement_widget.measurement_data = {
                'length': distance,
                'points': {
                    'P': point_p,
                    'R': point_r
                }
            }
            self.measurement_widget.update()

            return distance_meters
        
        except Exception as e:
            self.output_list.addItem(f"Error creating presized horizontal line: {str(e)}")

# =======================================================================================================================================
# Define the function for the Create Presized Vertical Line::
    def create_presized_vertical_line(self):
        """Create a new vertical line from point A to point C (same height as B) when Presized is clicked"""
        if not hasattr(self, 'measurement_points') or len(self.measurement_points) < 2:
            return
        
        try:
            # Get points A and B from the original measurement
            point_a = self.measurement_points[0]
            point_b = self.measurement_points[1]
            
            # Create point C at the same height as B but directly above A
            point_c = np.array([point_a[0], point_a[1], point_b[2]])
            
            # Calculate the distance between A and C (vertical height)
            distance_meters = np.linalg.norm(point_c - point_a)

            # Convert distance to current units
            units_suffix, conversion_factor = self.get_current_units()
            distance = distance_meters * conversion_factor
            
            self.presized_vertical_points = [point_a.tolist(), point_c.tolist()]  # [A, C]
            self.presized_height_meters = distance_meters 

            # Store for multiple saving
            if not hasattr(self, "all_presized_vertical_lines"):
                self.all_presized_vertical_lines = []

            self.all_presized_vertical_lines.append({
                "points": {"A": point_a.tolist(), "C": point_c.tolist()},
                "height_m": distance_meters
            })

            # 1. Change original AB line color from Red to LightGrey
            if hasattr(self, 'main_line_actor') and self.main_line_actor is not None:
                self.main_line_actor.GetProperty().SetColor(self.colors.GetColor3d("LightGrey"))

            # 2. Change point B sphere color to LightGrey
            if hasattr(self, 'point_b_actor') and self.point_b_actor is not None:
                self.point_b_actor.GetProperty().SetColor(self.colors.GetColor3d("LightGrey"))

            # 3. Change point B label color to LightGrey
            for actor in self.measurement_actors:
                if isinstance(actor, vtk.vtkFollower):
                    try:
                        text_source = actor.GetMapper().GetInputConnection(0, 0).GetProducer()
                        if isinstance(text_source, vtk.vtkVectorText) and text_source.GetText() == "B":
                            actor.GetProperty().SetColor(self.colors.GetColor3d("LightGrey"))
                            break
                    except:
                        continue

            # 4. Change distance label color to LightGrey
            if hasattr(self, 'distance_label_actor') and self.distance_label_actor is not None:
                self.distance_label_actor.GetProperty().SetColor(self.colors.GetColor3d("LightGrey"))

            # Draw the new vertical line from A to C in red
            self.presized_line_actor = self.add_line_between_points(point_a, point_c, "Red",f"AC={distance:.2f}{units_suffix}")
            
            # Add point C marker with label
            self.point_c_actor = self.add_sphere_marker(point_c, "C", color="Red")
            
            self.output_list.addItem(f"Presized Vertical Height AC: {distance:.2f} {units_suffix}")
            
            # Update the measurement data
            self.measurement_widget.measurement_data = {
                'length': distance,
                'points': {
                    'A': point_a,
                    'C': point_c
                }
            }
            self.measurement_widget.update()
            
            self.vtk_widget.GetRenderWindow().Render()

            return distance_meters
            
        except Exception as e:
            self.output_list.addItem(f"Error creating presized vertical line: {str(e)}")

# =======================================================================================================================================
#                                               ***** Measurement Ends ******
# =======================================================================================================================================


#                                           Menus and saving & loading part
# =======================================================================================================================================
    def add_layers_content(self):

        container = QFrame()
        layout = QVBoxLayout(container)
        container.setStyleSheet("""
            QFrame { 
                border: 2px solid #42A5F5; 
                border-radius: 10px; 
                background-color: #E3F2FD; 
                margin: 5px; 
            }
        """)

        self.measurement_layers_title = QLabel("Measurement Menu")
        self.measurement_layers_title.setAlignment(Qt.AlignCenter)
        self.measurement_layers_title.setStyleSheet("""
            font-weight: bold; 
            font-size: 13px; 
            padding: 8px; 
            background-color: #E3F2FD; 
            border-radius: 5px;
        """)
        layout.addWidget(self.measurement_layers_title)

        # --- RAILWAY MENU CHECKBOX ---
        self.railway_menu_checkbox = QCheckBox("Railway Menu")
        self.railway_menu_checkbox.setChecked(False)  # Default checked
        self.railway_menu_checkbox.setStyleSheet("""
            QCheckBox {
                font-weight: bold; 
                font-size: 16px; 
                color: white;
                background-color: rgba(52, 58, 64, 0.8);
                padding: 8px;
                border-radius: 4px;
            }
            QCheckBox::indicator {
                width: 16px;
                height: 16px;
            }
        """)
        layout.addWidget(self.railway_menu_checkbox)

        # --- RAILWAY MENU CONTENT CONTAINER --- ---
        self.railway_content_container = QWidget()
        self.railway_content_container.setVisible(False)
        self.railway_content_layout = QVBoxLayout(self.railway_content_container)
        self.railway_content_layout.setContentsMargins(0, 0, 0, 0)
        self.railway_content_layout.setSpacing(10)

        # --- ELECTRICAL SECTION ---
        self.electrical_section = self.create_checkbox_section(
            "Electrical",
            "electrical_container",
            [
                ("ohe_wire", "OHE Wire Measurements", "sub_checkbox"),
            ]
        )
        self.railway_content_layout.addWidget(self.electrical_section)

        # --- TRACK SECTION ---
        self.track_section = self.create_checkbox_section(
            "Track",
            "track_container"
        )
        self.railway_content_layout.addWidget(self.track_section)

        # --- CONSTRUCTION AND INFRASTRUCTURE SECTION ---
        self.construction_section = self.create_checkbox_section(
            "Construction and Infrastructure",
            "construction_container",
            [
                ("footover_bridge", "Footover Bridge Measurements", "sub_checkbox")
            ]
        )
        self.railway_content_layout.addWidget(self.construction_section)

        # --- SIGNAL SECTION ---
        self.signal_section = self.create_checkbox_section(
            "Signal and Telecom",
            "signal_container"
        )
        self.railway_content_layout.addWidget(self.signal_section)

        layout.addWidget(self.railway_content_container)

        # --- ROAD MENU CHECKBOX ---
        self.road_menu_checkbox = QCheckBox("Road Menu")
        self.road_menu_checkbox.setStyleSheet("""
            QCheckBox {
                font-weight: bold; 
                font-size: 16px; 
                color: white;
                background-color: rgba(52, 58, 64, 0.8);
                padding: 8px;
                border-radius: 4px;
            }
            QCheckBox::indicator {
                width: 16px;
                height: 16px;
            }
        """)
        layout.addWidget(self.road_menu_checkbox)

        # --- ROAD MENU CONTENT CONTAINER ---
        self.road_content_container = QWidget()
        self.road_content_container.setVisible(False)  # Hidden by default
        self.road_content_layout = QVBoxLayout(self.road_content_container)
        self.road_content_layout.setContentsMargins(0, 0, 0, 0)
        self.road_content_layout.setSpacing(10)

        # --- ROAD SECTION ---
        self.road_section = self.create_checkbox_section(
            "Road",
            "road_container"
        )
        self.road_content_layout.addWidget(self.road_section)

        # --- STRUCTURE SECTION ---
        self.structure_section = self.create_checkbox_section(
            "Structure",
            "structure_container",
            [
                ("toll_booth", "Toll Booth", "sub_checkbox"),
            ]
        )
        self.road_content_layout.addWidget(self.structure_section)

        # --- BRIDGE SECTION ---
        self.construction_section = self.create_checkbox_section(
            "Bridge",
            "bridge_container"
        )
        self.road_content_layout.addWidget(self.construction_section)

        layout.addWidget(self.road_content_container)
        layout.addStretch()

        # Connect checkbox signals
        self.railway_menu_checkbox.toggled.connect(self.toggle_railway_menu)
        self.road_menu_checkbox.toggled.connect(self.toggle_road_menu)

        return container
    
    def create_checkbox_section(self, section_text, container_name, items_list=None):
        """Create a section with main checkbox that can have sub-checkboxes"""
        section_widget = QWidget()
        section_layout = QVBoxLayout(section_widget)
        section_layout.setContentsMargins(3, 0, 0, 0)
        section_layout.setSpacing(0)

        # Create main section checkbox
        section_container = QWidget()
        section_container_layout = QHBoxLayout(section_container)
        section_container_layout.setContentsMargins(0, 0, 0, 0)
        
        section_checkbox = QCheckBox()
        section_checkbox.setFixedSize(35, 35)
        section_checkbox.setStyleSheet("""
            QCheckBox::indicator {
                width: 16px;
                height: 16px;
            }
        """)
        
        section_label = QLabel(section_text)
        section_label.setStyleSheet("""
            QLabel {
                color: black;
                font-weight: bold;
                font-size: 18px;
                border: none;
                padding: 0px;
            }
        """)
        section_label.setMinimumHeight(30)
        
        section_container_layout.addWidget(section_checkbox)
        section_container_layout.addWidget(section_label)
        section_layout.addWidget(section_container)

        # Create main button container for this section
        button_container = QWidget()
        button_container.setObjectName(f"{container_name}_container")
        button_container.setStyleSheet(f"""
            #{container_name}_container {{
                border: 1px solid black;
                background-color: #ffffff;
                margin-left: 35px;
            }}
        """)
        button_container.setVisible(False)  # Start hidden
        
        button_layout = QVBoxLayout(button_container)
        button_layout.setContentsMargins(20, 10, 10, 10)
        button_layout.setSpacing(0)

        # Add sub-checkboxes if any
        if items_list:
            for item_id, item_text, item_type in items_list:
                if item_type == "sub_checkbox":
                    # SIMPLE: Just create a basic sub-checkbox without nested containers
                    sub_checkbox = QCheckBox(item_text)
                    sub_checkbox.setStyleSheet("""
                        QCheckBox {
                            font-weight: bold;
                            font-size: 14px;
                            color: #2C3E50;
                            padding: 5px;
                        }
                    """)
                    button_layout.addWidget(sub_checkbox)
                    
                    # Connect sub-checkbox to show/hide its controls
                    # (You can add specific controls for each sub-checkbox later if needed)
                    sub_checkbox.toggled.connect(lambda checked, item_id=item_id: 
                                            self.on_sub_checkbox_toggled(item_id, checked))
        
        section_layout.addWidget(button_container)

        # Connect main checkbox to toggle the container
        section_checkbox.toggled.connect(lambda checked, bc=button_container: bc.setVisible(checked))
        
        # Store reference for this section
        section_data = {
            'widget': section_widget,
            'checkbox': section_checkbox,
            'label': section_label,
            'button_container': button_container
        }
        
        # Store based on section type
        if section_text == "Electrical":
            self.electrical_section_data = section_data
        elif section_text == "Track":
            self.track_section_data = section_data
        elif section_text == "Construction and Infrastructure":
            self.construction_section_data = section_data
        elif section_text == "Signal and Telecom":
            self.signal_section_data = section_data
        
        return section_widget
    
    def create_sub_checkbox(self, sub_id, sub_text, parent_layout):
        """Create a sub-checkbox that can have its own container"""
        sub_widget = QWidget()
        sub_layout = QHBoxLayout(sub_widget)
        sub_layout.setContentsMargins(0, 0, 0, 0)
        
        sub_checkbox = QCheckBox(sub_text)
        sub_checkbox.setStyleSheet("""
            QCheckBox {
                font-weight: bold;
                font-size: 14px;
                color: #2C3E50;
                padding: 5px;
            }
        """)
        
        sub_layout.addWidget(sub_checkbox)
        parent_layout.addWidget(sub_widget)
        
        # Create sub-container for this checkbox (initially hidden)
        sub_button_container = QWidget()
        sub_button_container.setObjectName(f"{sub_id}_sub_container")
        sub_button_container.setStyleSheet(f"""
            #{sub_id}_sub_container {{
                border: 1px solid #aaa;
                background-color: #f5f5f5;
                margin-left: 20px;
            }}
        """)
        sub_button_container.setVisible(False)
        
        sub_button_layout = QVBoxLayout(sub_button_container)
        sub_button_layout.setContentsMargins(25, 10, 10, 10)
        sub_button_layout.setSpacing(5)
        
        # Add items to sub-container based on sub_id
        # self.add_sub_checkbox_items(sub_id, sub_button_layout)
        
        parent_layout.addWidget(sub_button_container)
        
        # Connect sub-checkbox to toggle its container
        sub_checkbox.toggled.connect(lambda checked, sbc=sub_button_container: sbc.setVisible(checked))
        
        return sub_widget
    
    def reset_railway_menu_state(self):
       pass
    
    def reset_road_menu_state(self):
        pass

# =======================================================================================================================================  
# Toggle methods for Check boxes in Menu Panel 
    def toggle_railway_menu(self, checked):
        """Toggle railway menu visibility"""
        if checked:
            self.road_menu_checkbox.setChecked(False)
            self.railway_content_container.setVisible(True)
            self.road_content_container.setVisible(False)
            # Reset road menu state
            self.reset_road_menu_state()
        else:
            self.railway_content_container.setVisible(False)
            self.reset_railway_menu_state()

    def toggle_road_menu(self, checked):
        """Toggle road menu visibility"""
        if checked:
            self.railway_menu_checkbox.setChecked(False)
            self.road_content_container.setVisible(True)
            self.railway_content_container.setVisible(False)
            # Reset railway menu state
            self.reset_railway_menu_state()
        else:
            self.road_content_container.setVisible(False)
            self.reset_road_menu_state()

# =======================================================================================================================================
# Define the function to save measurements as a layer
    def Save_layers_measurements(self):
        if not hasattr(self, "point_cloud") or self.point_cloud is None:
            QMessageBox.warning(self, "Error", "No point cloud loaded.")
            return
        
        try:
            # --- 1. Fixed Base Folder Path ---
            base_measurement_path = os.path.join(self.WORKSHEETS_BASE_DIR, self.current_worksheet_name, "measurements")
            layer_folder = os.path.join(base_measurement_path, self.current_measurement_layer)
            os.makedirs(layer_folder, exist_ok=True)

            # --- 3. Use original file name if available ---
            if hasattr(self, "loaded_file_name") and self.loaded_file_name:
                base_name = self.loaded_file_name
            else:
                base_name = "unnamed_pointcloud"

            # --- 5. Save measurements JSON ---
            measurement_name = getattr(self, "current_measurement", "measurement")
            json_path = os.path.join(layer_folder, f"{base_name}_{measurement_name}.json")
            
            data = {"measurements": []}
            units_suffix, conversion_factor = self.get_current_units()

            if hasattr(self, "polygon_points") and len(self.polygon_points) >= 3:
                try:
                    edges = len(self.polygon_points)
                    entry = {
                        "type": "polygon",
                        "points": self.polygon_points,
                        "edges": edges,
                        "area": float(getattr(self, "polygon_area_meters", 0) * conversion_factor),
                        "perimeter": float(getattr(self, "polygon_perimeter_meters", 0) *conversion_factor),
                        "unit": units_suffix
                    }
                    
                    # Add height/depth/volume data if available
                    if hasattr(self, 'height_check') and self.height_check.isChecked() and self.height_input.text():
                        try:
                            entry["height"] = float(self.height_input.text())
                            entry["height_visualization"] = True
                        except ValueError:
                            pass
                            
                    if hasattr(self, 'depth_check') and self.depth_check.isChecked() and self.depth_input.text():
                        try:
                            entry["depth"] = float(self.depth_input.text())
                            entry["depth_visualization"] = True
                        except ValueError:
                            pass
                            
                    if hasattr(self, 'volume_check') and self.volume_check.isChecked():
                        entry["volume_calculated"] = True
                        
                    data["measurements"].append(entry)
                except Exception as e:
                    self.output_list.addItem(f"⚠️ Error saving polygon: {e}")

            # Save Round Pillar Polygon
            if hasattr(self, "round_pillar_points") and len(self.round_pillar_points) >= 3:
                try:
                    edges = len(self.round_pillar_points)
                    entry = {
                        "type": "round_pillar_polygon",
                        "points": self.round_pillar_points,
                        "edges": edges,
                        "surface_area": float(getattr(self, "round_pillar_surface_area_meters", 0) * conversion_factor),
                        "perimeter": float(getattr(self, "round_pillar_perimeter_meters", 0) * conversion_factor),
                        "unit": units_suffix
                    }
                    data["measurements"].append(entry)
                except Exception as e:
                    self.output_list.addItem(f"⚠️ Error saving round pillar polygon: {e}")

            # Save OHE Pole Angle with rail level
            if hasattr(self, "ohe_pole_angle_points") and len(self.ohe_pole_angle_points) >=3:
                try:
                    if len(self.ohe_pole_angle_points) >=3:
                        top_pt = self.ohe_pole_angle_points[0].tolist() 
                        inter_pt = self.ohe_pole_angle_points[1].tolist()
                        track_pt = self.ohe_pole_angle_points[2].tolist()
                        left_rail = np.array(self.left_point).tolist() if hasattr(self, "left_point") else None
                        right_rail = np.array(self.right_point).tolist() if hasattr(self, "right_point") else None

                        entry = {
                            "type": "ohe_pole_angle_with_rail",
                            "points": {
                                "Top": top_pt,
                                "Intersection": inter_pt,
                                "Track": track_pt
                            },
                            "height": float(self.ohe_pole_height_meters),
                            "angle_deg": float(self.ohe_pole_angle_with_rail),
                            "left_rail": left_rail,
                            "right_rail": right_rail,
                            "MTOR": float(self.OHE_baseline_z),
                            "unit": units_suffix
                        }
                        data["measurements"].append(entry)
                except Exception as e:
                    QMessageBox.critical(self, "Error", f"Error Saving OHE Pole Angle with Rail: {str(e)}")

            # All Angles Measurement 
            if hasattr(self, "all_angles_points") and len(self.all_angles_points) == 3:
                try:
                    p1, p2, p3 = [np.array(p).tolist() for p in self.all_angles_points]
                    angle_deg = float(getattr(self, "stored_all_angles", 0.0))

                    entry = {
                        "type": "all_angles",
                        "points": {"A": p1, "B": p2, "C": p3},
                        "angle_deg": angle_deg,
                        "unit": units_suffix
                    }
                    data["measurements"].append(entry)
                except Exception as e:
                    QMessageBox.critical(self, "Error", f"Error Saving All Angles: {str(e)}")

            # --- distance between catenary to rail and contact to rail ---
            if hasattr(self, "last_curve_points") and len(self.last_curve_points) >= 3:
                try:
                    left_rail= np.array(self.left_point).tolist() if hasattr(self, "left_point") else None
                    right_rail = np.array(self.right_point).tolist() if hasattr(self, "right_point") else None
                    entry = {
                        "type": "distance_between_catenary_to_raillevel",
                        "points": {
                            "control_points": [list(map(float, p)) for p in self.last_curve_points],
                            "sampled_points": [list(map(float, s)) for s in getattr(self, "last_sampled_points", [])]
                        },
                        "angles": {
                            "start_interior": float(getattr(self, "last_interior_start", 0)),
                            "start_exterior": float(getattr(self, "last_exterior_start", 0)),
                            "end_interior": float(getattr(self, "last_interior_end", 0)),
                            "end_exterior": float(getattr(self, "last_exterior_end", 0))
                        },
                        "distance": float(getattr(self, "last_baseline_len", 0)),
                        "labels": getattr(self, "last_angle_labels", []),
                        "left_rail": left_rail,
                        "right_rail": right_rail,
                        "MTOR": float(self.OHE_baseline_z),
                        "unit": units_suffix
                    }
                    data["measurements"].append(entry)
                except Exception as e:
                    QMessageBox.critical(self, "Error", f"Error saving curve measurement: {str(e)}")

            if hasattr(self, "last_curve_contact_rail_points") and len(self.last_curve_contact_rail_points) >= 3:
                try:
                    left_rail= np.array(self.left_point).tolist() if hasattr(self, "left_point") else None
                    right_rail = np.array(self.right_point).tolist() if hasattr(self, "right_point") else None
                    entry = {
                        "type": "distance_between_catenary_to_raillevel",
                        "points": {
                            "control_points": [list(map(float, p)) for p in self.last_curve_contact_rail_points],
                            "sampled_points": [list(map(float, s)) for s in getattr(self, "last_sampled_points", [])]
                        },
                        "angles": {
                            "start_interior": float(getattr(self, "last_interior_start", 0)),
                            "start_exterior": float(getattr(self, "last_exterior_start", 0)),
                            "end_interior": float(getattr(self, "last_interior_end", 0)),
                            "end_exterior": float(getattr(self, "last_exterior_end", 0))
                        },
                        "distance": float(getattr(self, "last_baseline_len", 0)),
                        "labels": getattr(self, "last_angle_labels", []),
                        "left_rail": left_rail,
                        "right_rail": right_rail,
                        "MTOR": float(self.OHE_baseline_z),
                        "unit": units_suffix
                    }
                    data["measurements"].append(entry)
                except Exception as e:
                    QMessageBox.critical(self, "Error", f"Error saving curve measurement: {str(e)}")

            # --- distance between catenary to contact ---
            if hasattr(self, "last_catenary_contact_curve_points") and len(self.last_catenary_contact_curve_points) >= 3:
                try:
                    contact_wire_points = (
                        [list(map(float, p)) for p in getattr(self, "contact_wire_points", [])]
                        if hasattr(self, "contact_wire_points") and len(self.contact_wire_points) >= 2
                        else [])
                    entry = {
                        "type": "distance_between_catenary_to_contact",
                        "points": {
                            "control_points": [list(map(float, p)) for p in self.last_catenary_contact_curve_points],
                            "sampled_points": [list(map(float, s)) for s in getattr(self, "last_sampled_points", [])],
                            "contact_wire_points": contact_wire_points,
                        },
                        "angles": {
                            "start_interior": float(getattr(self, "last_interior_start", 0)),
                            "start_exterior": float(getattr(self, "last_exterior_start", 0)),
                            "end_interior": float(getattr(self, "last_interior_end", 0)),
                            "end_exterior": float(getattr(self, "last_exterior_end", 0))
                        },
                        "distance": float(getattr(self, "last_baseline_len", 0)),
                        "labels": getattr(self, "last_angle_labels", []),
                        "unit": units_suffix
                    }
                    data["measurements"].append(entry)
                except Exception as e:
                    QMessageBox.critical(self, "Error", f"Error saving distance between catenary to contact wire measurement layer: {str(e)}")

            # --- Defect Angle Measurements ---
            if hasattr(self, "all_defect_angle_measurements") and self.all_defect_angle_measurements:
                try:
                    for defect_entry in self.all_defect_angle_measurements:
                        entry = {
                            "type": "defect_angle",
                            "points": defect_entry["points"],
                            "angle_deg": defect_entry["angle_deg"],
                            "label": defect_entry["label"],
                            "unit": units_suffix
                        }
                        data["measurements"].append(entry)
                except Exception as e:
                    QMessageBox.critical(self, "Error", f"Error saving Defect Angle measurements: {str(e)}")

            # # --- Save Multiple Vertical Lines ---
            # if hasattr(self, "all_vertical_lines") and self.all_vertical_lines:
            #     try:
            #         for vline in self.all_vertical_lines:
            #             entry = {
            #                 "type": "vertical_line",
            #                 "points": vline["points"],
            #                 "height": float(vline.get("height", 0)),
            #                 "unit": vline.get("unit", "m")
            #             }
            #             data["measurements"].append(entry)
            #     except Exception as e:
            #         QMessageBox.critical(self, "Error", f"Error saving vertical line: {str(e)}")

            # # --- Save Multiple Horizontal Lines ---
            # if hasattr(self, "all_horizontal_lines") and self.all_horizontal_lines:
            #     try:
            #         for hline in self.all_horizontal_lines:
            #             entry = {
            #                 "type": "horizontal_line",
            #                 "points": hline["points"],
            #                 "length": float(hline.get("length", 0)),
            #                 "unit": hline.get("unit", "m")
            #             }
            #             data["measurements"].append(entry)
            #     except Exception as e:
            #         QMessageBox.critical(self, "Error", f"Error saving horizontal line: {str(e)}")

            # --- Save Multiple Presized Vertical Lines ---
            if hasattr(self, "all_presized_vertical_lines") and self.all_presized_vertical_lines:
                try:
                    for v_entry in self.all_presized_vertical_lines:
                        A = v_entry["points"]["A"]
                        C = v_entry["points"]["C"]
                        height_m = v_entry.get("height_m", 0)
                        volume = v_entry.get("volume", 0)
                        outer_surface = v_entry.get("outer_surface", 0)
                        entry = {
                            "type": "presized_vertical",
                            "points": {"A": A, "C": C},
                            "height": float(height_m * conversion_factor),
                            "volume": float(volume),
                            "outer_surface": float(outer_surface),
                            "unit": units_suffix
                        }
                        data["measurements"].append(entry)
                except Exception as e:
                    QMessageBox.critical(self, "Error", f"Error saving presized vertical: {str(e)}")

            # --- Save Multiple Presized Horizontal Lines ---
            if hasattr(self, "all_presized_horizontal_lines") and self.all_presized_horizontal_lines:
                try:
                    for h_entry in self.all_presized_horizontal_lines:
                        P = h_entry["points"]["P"]
                        R = h_entry["points"]["R"]
                        length_m = h_entry.get("length_m", 0)
                        volume = h_entry.get("volume", 0)
                        outer_surface = h_entry.get("outer_surface", 0)
                        entry = {
                            "type": "presized_horizontal",
                            "points": {"P": P, "R": R},
                            "length": float(length_m * conversion_factor),
                            "volume": float(volume),
                            "outer_surface": float(outer_surface),
                            "unit": units_suffix
                        }
                        data["measurements"].append(entry)
                except Exception as e:
                    QMessageBox.critical(self, "Error", f"Error saving presized horizontal: {str(e)}")

            # --- Combined Pillar Totals (if available) ---
            if (hasattr(self, "total_pillar_volume_meters") and 
                hasattr(self, "total_pillar_outer_surface_meters") and
                self.total_pillar_volume_meters > 0.0 and
                self.total_pillar_outer_surface_meters > 0.0):
                try:
                    centroid = None

                    # Determine centroid if polygon points exist
                    if (hasattr(self, "round_pillar_points") and 
                        hasattr(self, "polygon_points") and 
                        len(self.round_pillar_points) >= 3 and
                        len(self.polygon_points) >= 3):

                        all_points = np.vstack((self.round_pillar_points, self.polygon_points))
                        centroid = np.mean(all_points, axis=0).tolist()

                    # JSON entry for pillar dimensions
                    entry = {
                        "m_type": "pillar_dimension",
                        "total_volume_m": float(self.total_pillar_volume_meters * conversion_factor),
                        "total_outer_surface_m": float(self.total_pillar_outer_surface_meters * conversion_factor),
                        "centroid": centroid,
                        "unit": units_suffix
                    }

                    data["measurements"].append(entry)

                except Exception as e:
                    QMessageBox.critical(self, "Error", f"Error saving pillar dimensions: {str(e)}")
            
            with open(json_path, "w") as f:
                json.dump(data, f, indent=4)

            QMessageBox.information(self, "Success", f"✅ Measurement layer saved to {json_path}")

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error saving measurement layer: {str(e)}")

    def clear_2d_layer_panel(self):
        # Clear 2D layers panel
        if hasattr(self, 'two_D_layers_layout') and self.two_D_layers_layout:
            while self.two_D_layers_layout.count():
                item = self.two_D_layers_layout.takeAt(0)
                if item.widget():
                    item.widget().deleteLater()
            self.two_D_layers_layout.addStretch(1)
    
    def clear_3d_layer_panel(self):
        # Clear 3D layers panel
        if hasattr(self, 'three_D_layers_layout') and self.three_D_layers_layout:
            while self.three_D_layers_layout.count():
                item = self.three_D_layers_layout.takeAt(0)
                if item.widget():
                    item.widget().deleteLater()
            self.three_D_layers_layout.addStretch(1)

# ============================= Reset Functionalities =============================

# =======================================================================================================================================  
    def reset_action(self):
        """Reset the action section, measurement points, and output"""
        # Clear measurement points and actors
        self.measurement_points = []
        self.line_angle = None

        # Clear measurement line points if they exist
        if hasattr(self, 'measurement_line_points'):
            del self.measurement_line_points

        # Remove all measurement actors except digging points (blue spheres)
        # Remove only measurement actors (not baseline actors)
        actors_to_remove = []
        for actor in self.measurement_actors:
            # Skip actors that are in baseline_actors
            if actor not in self.baseline_actors:
                actors_to_remove.append(actor)

        # Remove the actors
        for actor in actors_to_remove:
            self.renderer.RemoveActor(actor)
            if actor in self.measurement_actors:
                self.measurement_actors.remove(actor)

        # Also remove baseline actors if they exist
        if hasattr(self, 'baseline_actors'):
            for actor in self.baseline_actors:
                self.renderer.RemoveActor(actor)
                if actor in self.measurement_actors:
                    self.measurement_actors.remove(actor)
            self.baseline_actors = []  # Clear the baseline actors list

        # Reset measurement widget
        if hasattr(self, 'measurement_widget'):
            self.measurement_widget.clear()

        # Hide surface controls
        self.hide_surface_controls()

        if self.filling_check.isChecked():
            self.depth_check.setChecked(False)
            self.height_check.setChecked(False)
            self.volume_check.setChecked(False)
            self.depth_input.clear()
            self.height_input.clear()

        self.presized_button.setVisible(False)

        self.complete_polygon_button.setVisible(False)
        self.complete_polygon_button.setStyleSheet("")  # Reset to default style

        # Clear the output list
        if hasattr(self, 'output_list'):
            self.output_list.clear()

        if hasattr(self, 'depth_input'):
            self.depth_input.clear()
            self.depth_input.setVisible(False)

        # Reset digging point inputs and clear dropdowns
        if hasattr(self, 'digging_point_input'):
            # Clear all digging points data
            self.digging_point_input.digging_points = []
            self.digging_point_input.digging_point_combo.clear()
            self.digging_point_input.polygon_point_combo.clear()
            self.digging_point_input.setVisible(False)
            
            # Clear digging points info if it exists
            if hasattr(self, 'digging_points_info'):
                del self.digging_points_info

        # Clear connection dropdowns and hide the group
        if hasattr(self, 'connection_group'):
            self.from_digging_combo.clear()
            self.to_digging_combo.clear()
            self.connection_group.setVisible(False)

        # Clear Polygon-Digging Connect section
        if hasattr(self, 'polygon_digging_group'):
            self.polygon_point_combo.clear()
            self.digging_point_combo.clear()
            self.polygon_digging_group.setVisible(False)

        # Clear surface visuals
        self.clear_surface_visuals()

            # Reset surface selection if it exists
        if hasattr(self, 'surface_selection_group'):
            self.surface_selection_group.setVisible(False)

        # Clear the surface selection dropdown
        if hasattr(self, 'surface_combo'):
            self.surface_combo.clear()
            #self.surface_selection_group.setVisible(False)

        # Uncheck all checkboxes
        if hasattr(self, 'length_check'):
            self.depth_check.setChecked(False)
            self.volume_check.setChecked(False)

        # Reset view and plotting states
        self.freeze_view = False
        self.plotting_active = True

        # Ensure interactor style is restored
        if hasattr(self, 'vtk_widget'):
            interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
            style = vtkInteractorStyleTrackballCamera()
            interactor.SetInteractorStyle(style)

        # Clear cropped data
        if hasattr(self, 'cropped_cloud'):
            self.cropped_cloud = None
    
        # Disable save crop button
        if hasattr(self, 'save_crop_button'):
            self.save_crop_button.setEnabled(False)

        if hasattr(self, 'presized_line_actor'):
            self.renderer.RemoveActor(self.presized_line_actor)
            if self.presized_line_actor in self.measurement_actors:
                self.measurement_actors.remove(self.presized_line_actor)
        if hasattr(self, 'point_c_actor'):
            self.renderer.RemoveActor(self.point_c_actor)
            if self.point_c_actor in self.measurement_actors:
                self.measurement_actors.remove(self.point_c_actor)

        # Close crop window if open
        if hasattr(self, 'crop_window'):
            try:
                self.crop_window.close()
            except:
                pass

        self.current_measurement = None
        if hasattr(self, 'vtk_widget'):
            self.vtk_widget.GetRenderWindow().Render()

        # for polygon and its volume 
        self.polygon_points = [] 
        self.polygon_actors = []
        self.polygon_area_meters = 0.0
        self.polygon_perimeter_meters = 0.0
        self.polygon_volume_meters = 0.0
        self.polygon_outer_surface_meters = 0.0
        
        # Reset saving the line actor, points actor
        self.vertical_points = []
        self.vertical_height_meters = 0.0  
        self.main_line_actor = None
        self.point_a_actor = None
        self.point_b_actor = None
        self.presized_vertical_points = [] 
        self.presized_height_meters = 0.0
        self.presized_line_actor = None
        self.point_c_actor = None

        # reset actors of horizontal line measurement
        self.horizontal_points = [] 
        self.horizontal_length_meters = 0.0
        self.distance_label_actor = None
        self.horizontal_line_actor = None
        self.point_p_actor = None
        self.point_q_actor = None
        self.horizontal_distance_label_actor = None
        self.presized_horizontal_points = []
        self.presized_horizontal_length_meters = 0.0
        self.presized_horizontal_line_actor = None
        self.point_r_actor = None

        # for round pillar polygon and its volume
        self.round_pillar_surface_area_meters = 0.0 
        self.round_pillar_perimeter_meters = 0.0
        self.round_pillar_outer_surface_meters = 0.0
        self.round_pillar_volume_meters = 0.0
        self.round_pillar_points = []
        self.round_pillar_actors = []
        self.presized_volume = 0.0                    # store presized volume meters
        self.presized_outer_surface = 0.0             # store presized outer surface meters

        # for pillars total volume
        self.total_pillar_volume_meters = 0.0
        self.total_pillar_outer_surface_meters = 0.0
        self.ohe_pole_angle_points = []
        self.left_point = None
        self.right_point = None
        self.ohe_pole_angle_with_rail = 0.0
        self.ohe_pole_height_meters = 0.0

        self.all_angles_points = []
        self.stored_all_angles = 0.0

        self.last_curve_points = []
        self.last_catenary_contact_curve_points = []
        self.last_curve_contact_rail_points = []
        self.last_sampled_points = []
        self.last_interior_start = 0.0
        self.last_exterior_start = 0.0
        self.last_interior_end = 0.0
        self.last_exterior_end = 0.0
        self.last_baseline_len = 0.0
        self.last_angle_labels = []

        self.defect_angle_points = []
        self.defect_angle = 0.0

        self.all_defect_angle_measurements = []
        self.all_presized_vertical_lines = []
        self.all_presized_horizontal_lines = []

        # ------------Visualization App - reset_actions -------------------

        """Reset the current active line type's drawings from the graph"""
        if self.active_line_type:
            # Store the active line type for the message
            active_type_name = self.active_line_type
            
            # Clear current drawing session for the active line type
            self.current_points = []
            if self.current_artist is not None:
                try:
                    self.current_artist.remove()
                except:
                    pass
                self.current_artist = None
            self.current_redo_points = []
            
            # Clear current point labels for the active line type
            for label in self.current_point_labels:
                try:
                    if label in self.ax.texts:
                        label.remove()
                except:
                    pass
            self.current_point_labels = []
            
            # If we have any finished polylines for this active line type, remove them
            if active_type_name in self.line_types:
                # Remove all artists for this line type
                for artist in self.line_types[active_type_name]['artists']:
                    try:
                        if artist in self.ax.lines or artist in self.ax.collections:
                            artist.remove()
                    except:
                        pass
                # Clear the lists
                self.line_types[active_type_name]['artists'] = []
                self.line_types[active_type_name]['polylines'] = []
                
                # Also remove from all_graph_lines
                new_all_graph_lines = []
                for item in self.all_graph_lines:
                    lt, points, artist, ann = item
                    if lt != active_type_name:
                        new_all_graph_lines.append(item)
                    else:
                        # Remove the annotation if it exists
                        if ann:
                            try:
                                ann.remove()
                            except:
                                pass
                self.all_graph_lines = new_all_graph_lines
            
            # Also remove any point labels for this line type
            if active_type_name == 'construction_dots':
                # For construction dots, remove the P1, P2 labels
                texts_to_remove = []
                for text in self.ax.texts:
                    if hasattr(text, 'point_data') and text.point_data.get('line_type') == 'construction_dots':
                        texts_to_remove.append(text)
                
                for text in texts_to_remove:
                    try:
                        text.remove()
                    except:
                        pass
            
            # Redraw canvas
            self.canvas.draw()
            self.figure.tight_layout()
            
            # Reset the checkbox for this line type
            if active_type_name == 'construction':
                self.construction_line.setChecked(False)
            elif active_type_name == 'surface':
                self.surface_baseline.setChecked(False)
            elif active_type_name == 'road_surface':
                self.road_surface_line.setChecked(False)
            elif active_type_name == 'construction_dots':
                self.construction_dots_line.setChecked(False)
            elif active_type_name == 'deck_line':
                self.deck_line.setChecked(False)
            elif active_type_name == 'projection_line':
                self.projection_line.setChecked(False)
            elif active_type_name == 'zero':
                self.zero_line.setChecked(False)
            
            # Format the message
            display_name = active_type_name.replace('_', ' ').title()
            self.message_text.append(f"Reset {display_name} drawings")
            
            # Reset active line type AFTER using it
            self.active_line_type = None
            
            # Disconnect drawing events if connected
            if self.cid_click is not None:
                self.canvas.mpl_disconnect(self.cid_click)
                self.cid_click = None
            if self.cid_key is not None:
                self.canvas.mpl_disconnect(self.cid_key)
                self.cid_key = None
        else:
            self.message_text.append("No active line type to reset. Please select a line type first.")
            
        # Clear all curve labels
        if hasattr(self, 'curve_labels'):
            for label in self.curve_labels:
                try:
                    label.remove()
                except:
                    pass
            self.curve_labels = []

        if self.curve_pick_id:
            self.canvas.mpl_disconnect(self.curve_pick_id)
            self.curve_pick_id = None

        self.curve_active = False
        self.current_curve_text = ""
        self.curve_start_x = None

        # Reset button
        self.preview_button.setText("Curve")
        self.preview_button.setStyleSheet("""
            QPushButton { background-color: #808080; color: white; padding: 12px; border-radius: 6px;
                          font-size: 15px; font-weight: bold; }
            QPushButton:hover { background-color: #6E6E6E; }
        """)

# =======================================================================================================================================
# Define the function for the reset all:
    def reset_all(self):
        """Reset ALL lines from the graph"""
        # First call reset_action to handle any current drawing
        self.reset_action()

        # Clear any temporary zero line drawing actors
        if hasattr(self, 'temp_zero_actors'):
            for actor in self.temp_zero_actors:
                self.renderer.RemoveActor(actor)
            self.temp_zero_actors = []

        # Reset slider position
        if hasattr(self, 'volume_slider'):
            self.volume_slider.setValue(0)

        # Reset camera to full view
        if hasattr(self, 'focus_camera_on_full_cloud'):
            self.focus_camera_on_full_cloud()

        self.remove_slider_marker()
        self.clear_2d_layer_panel()
        self.clear_3d_layer_panel()

        self.slider_marker_actor = None

        self.worksheet_display.setVisible(False)

        if self.checkboxes.isVisible():
            self.checkboxes.setVisible(False)

        self.mode_banner.setVisible(False)

        self.current_worksheet_name = None

        self.main_measurement_section.setVisible(False)

        self.two_D_frame.setVisible(False)
        self.three_D_frame.setVisible(False)

        self.scale_section.setVisible(False)
        
        # Now remove ALL lines from all line types
        for line_type in self.line_types:
            # Remove all artists for this line type
            for artist in self.line_types[line_type]['artists']:
                try:
                    if artist and hasattr(artist, 'remove'):
                        artist.remove()
                except Exception as e:
                    print(f"Error removing artist for {line_type}: {e}")
            
            # Clear the lists
            self.line_types[line_type]['artists'] = []
            self.line_types[line_type]['polylines'] = []
        
        # Clear all_graph_lines
        for lt, points, artist, ann in self.all_graph_lines:
            try:
                if artist and hasattr(artist, 'remove'):
                    artist.remove()
            except:
                pass
            try:
                if ann and hasattr(ann, 'remove'):
                    ann.remove()
            except:
                pass
        
        self.all_graph_lines = []
        self.redo_stack = []
        
        # Clear ALL point labels (not just current ones)
        for label in self.point_labels:
            try:
                if label and label in self.ax.texts:
                    label.remove()
            except:
                pass
        self.point_labels = []
        
        # Clear ALL current point labels
        for label in self.current_point_labels:
            try:
                if label and label in self.ax.texts:
                    label.remove()
            except:
                pass
        self.current_point_labels = []
        
        # Also clear any remaining text objects (including construction dots labels)
        texts_to_remove = []
        for text in self.ax.texts:
            # Keep only the annotation text
            if text != self.annotation:
                texts_to_remove.append(text)
        
        for text in texts_to_remove:
            try:
                text.remove()
            except:
                pass
        
        # Reset zero line if it exists
        if self.zero_line_set:
            if self.zero_graph_line:
                try:
                    self.zero_graph_line.remove()
                except:
                    pass
            
            # Reset zero line variables but keep the line set
            self.zero_graph_line = None
            
            # Redraw the zero line if checkbox is checked
            if self.zero_line.isChecked():
                self.zero_graph_line, = self.ax.plot([0, self.total_distance], [0, 0], 
                                                    color='purple', linewidth=3)
        
        # Uncheck all checkboxes
        self.construction_line.setChecked(False)
        self.surface_baseline.setChecked(False)
        if hasattr(self, 'zero_line'):
            self.zero_line.setChecked(False)
        if hasattr(self, 'road_surface_line'):
            self.road_surface_line.setChecked(False)
        if hasattr(self, 'construction_dots_line'):
            self.construction_dots_line.setChecked(False)
        if hasattr(self, 'deck_line'):
            self.deck_line.setChecked(False)
        if hasattr(self, 'projection_line'):
            self.projection_line.setChecked(False)
        
        # ===== ADD THIS SECTION: Hide road/bridge baseline checkboxes and reset button names =====
        # Reset road baseline

        self.road_surface_container.setVisible(False)
        self.surface_container.setVisible(False)
        self.zero_container.setVisible(False)
        self.construction_container.setVisible(False)
        # Hide bottom section on full reset
        self.bottom_section.setVisible(False)
        
        self.deck_line_container.setVisible(False)
        self.projection_container.setVisible(False)
        if hasattr(self, 'bridge_zero_container'):
            self.bridge_zero_container.setVisible(False)
        if hasattr(self, 'construction_dots_container'):
            self.construction_dots_container.setVisible(False)
        
        # Hide the additional buttons
        self.preview_button.setVisible(False)
        self.elivation_angle_button.setVisible(False)
        self.threed_map_button.setVisible(False)
        self.save_button.setVisible(False)
        
        # Reset active line type and drawing state
        self.active_line_type = None
        self.current_points = []
        if self.current_artist is not None:
            try:
                self.current_artist.remove()
            except:
                pass
            self.current_artist = None
        
        # Disconnect drawing events if connected
        if self.cid_click is not None:
            self.canvas.mpl_disconnect(self.cid_click)
            self.cid_click = None
        if self.cid_key is not None:
            self.canvas.mpl_disconnect(self.cid_key)
            self.cid_key = None
        
        # Reset scale section
        if self.zero_line_set:
            # Keep zero line set but update display
            self.update_chainage_ticks()
            if hasattr(self, 'scale_section'):
                self.scale_section.setVisible(False)
        else:
            # Hide scale section
            if hasattr(self, 'scale_section'):
                self.scale_section.setVisible(False)
        
        # Redraw canvas
        self.canvas.draw()
        self.figure.tight_layout()

        self.clear_baseline_planes()
        
        # Also reset 3D measurements if needed
        self.message_text.append("All graph lines have been reset")
        
        # Reset curve-related attributes
        if hasattr(self, 'curve_annotation'):
            try:
                if self.curve_annotation and self.curve_annotation in self.ax.texts:
                    self.curve_annotation.remove()
            except:
                pass
            self.curve_annotation = None
        
        if hasattr(self, 'curve_arrow_annotation'):
            try:
                if self.curve_arrow_annotation and self.curve_arrow_annotation in self.ax.patches:
                    self.curve_arrow_annotation.remove()
            except:
                pass
            self.curve_arrow_annotation = None
        
        if hasattr(self, 'curve_pick_id') and self.curve_pick_id:
            try:
                self.canvas.mpl_disconnect(self.curve_pick_id)
            except:
                pass
            self.curve_pick_id = None
        
        if hasattr(self, 'curve_annotation_x_pos'):
            self.curve_annotation_x_pos = None
        
        if hasattr(self, 'current_curve_config'):
            self.current_curve_config = {'outer_curve': False, 'inner_curve': False, 'angle': 0.0}
        
        if hasattr(self, 'preview_button'):
            self.preview_button.setText("Curve")
            self.preview_button.setStyleSheet("""
                QPushButton {
                    background-color: #808080;
                    color: white;
                    border: none;
                    padding: 10px;
                    border-radius: 5px;
                    font-size: 14px;
                    font-weight: bold;
                }
                QPushButton:hover { background-color: #6E6E6E; }
                QPushButton:pressed { background-color: #5A5A5A; }
            """)

        # Render the VTK window
        if hasattr(self, 'vtk_widget'):
            self.vtk_widget.GetRenderWindow().Render()

        # Clear all curve labels
        if hasattr(self, 'curve_labels'):
            for label in self.curve_labels:
                try:
                    label.remove()
                except:
                    pass
            self.curve_labels = []

        if self.curve_pick_id:
            self.canvas.mpl_disconnect(self.curve_pick_id)
            self.curve_pick_id = None

        self.curve_active = False
        self.current_curve_text = ""
        self.curve_start_x = None

        # Clear 3D curve actors
        for actor in self.curve_3d_actors:
            self.renderer.RemoveActor(actor)
        self.curve_3d_actors.clear()
        self.curve_start_point_3d = None

        # Re-render the vtk window
        self.vtk_widget.GetRenderWindow().Render()

        # Clear reference actors using the fixed method
        self.clear_reference_actors()
        
        # Also clear any stored lists
        if hasattr(self, 'reference_actors'):
            self.reference_actors = []

        if hasattr(self, 'herarchy_section'):
            self.herarchy_section.setVisible(False)
            self.clear_hierarchy_section_completely()

        self.right_section.setMinimumHeight(900)

        self.clear_merger_markers()
        self.merger_markers = []
        self.message_text.clear()

#------------------ Measurement_reset_all -------------------------------
        self.freeze_view = False
        self.plotting_active = True
        
        self.loaded_file_name = None
        self.loaded_file_path = None

        actors_to_remove = []
        for actor in self.measurement_actors:
            actors_to_remove.append(actor)
        for actor in actors_to_remove:
            self.renderer.RemoveActor(actor)
            if actor in self.measurement_actors:
                self.measurement_actors.remove(actor)
        # Reset point cloud
        if self.point_cloud_actor:
            self.renderer.RemoveActor(self.point_cloud_actor)
            self.point_cloud_actor = None
            self.point_cloud = None

        self.railway_menu_checkbox.setChecked(False)
        self.road_menu_checkbox.setChecked(False)

        self.railway_content_container.setVisible(False)
        self.road_content_container.setVisible(False)

        # Then clear baseline if desired
        for actor in self.baseline_actors:
            self.renderer.RemoveActor(actor)
        self.baseline_actors = []

        # Reset UI state
        self.measurement_active = False
        self.measurement_group.setEnabled(False)
        self.action_group.setEnabled(False)
        self.start_button.setEnabled(True)

        # Clear baseline actors if they exist
        if hasattr(self, 'baseline_actors'):
            for actor in self.baseline_actors:
                self.renderer.RemoveActor(actor)
            self.baseline_actors = []
        
        # Reset measurement widget
        if hasattr(self, 'measurement_widget'):
            self.measurement_widget.clear()
        
        # Reset digging points
        if hasattr(self, 'digging_point_input'):
            self.digging_point_input.reset_digging_points()
        
        # Reset digging points info
        if hasattr(self, 'digging_points_info'):
            del self.digging_points_info

        # Reset surface info
        if hasattr(self, 'surface_info'):
            del self.surface_info

        # Uncheck all checkboxes in the Action section
        self.measurement_check.setChecked(False)
        self.filling_check.setChecked(False)
        self.cutting_check.setChecked(False)
        self.extraction_check.setChecked(False)
        self.railway_measurement_check.setChecked(False)

        # Hide measurement controls
        self.hide_measurement_controls()

         # Clear output list
        self.output_list.clear()

        # Reset the style to default
        self.measurement_group.setStyleSheet("""
                                            QGroupBox {
                                            border: 1px solid gray;
                                            border-radius: 5px;
                                            margin: 0.2em;
                                            margin-top: 0.5em;
                                            }
                                            QGroupBox::title {
                                            subcontrol-origin: margin;
                                            left: 10px;
                                            padding: 0 3px;
                                            }
                                            """)
    
        # Reset File Load section style
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

# =======================================================================================================================================
    def calculate_cut_volume_with_width(self, polygon_points, reference_data, road_width=None):
        """Calculate cut volume considering road width from reference baseline"""
        try:
            import numpy as np
            
            # Extract reference points
            if isinstance(reference_data[0], dict):
                ref_points = np.array([p['world_coordinates'] for p in reference_data 
                                    if 'world_coordinates' in p])
            else:
                ref_points = np.array(reference_data)
        
            if len(ref_points) == 0:
                return None
            
            # Get road width from reference data if not provided
            if road_width is None:
                # Try to get width from reference data structure
                if hasattr(self, 'current_curve_config') and 'width' in self.current_curve_config:
                    road_width = self.current_curve_config['width']
                else:
                    # Default width
                    road_width = 10.0
            
            # Get polygon points
            poly_points = np.array(polygon_points)
            
            # Find the reference segment that aligns with polygon
            # For simplicity, find the reference line segment closest to polygon center
            center_x = np.mean(poly_points[:, 0])
            center_y = np.mean(poly_points[:, 1])
            
            # Find closest reference point
            distances = np.sqrt(
                (ref_points[:, 0] - center_x)**2 + 
                (ref_points[:, 1] - center_y)**2
            )
            closest_idx = np.argmin(distances)
            
            # Get the segment containing this point
            if closest_idx < len(ref_points) - 1:
                segment_start = ref_points[closest_idx]
                segment_end = ref_points[closest_idx + 1]
            else:
                segment_start = ref_points[closest_idx - 1]
                segment_end = ref_points[closest_idx]
            
            # Calculate road direction vector
            road_dir = segment_end[:2] - segment_start[:2]
            road_length = np.linalg.norm(road_dir)
            if road_length > 0:
                road_dir = road_dir / road_length
            
            # Calculate perpendicular direction for width
            perp_dir = np.array([-road_dir[1], road_dir[0]])
            
            # Create road boundaries (left and right edges)
            half_width = road_width / 2
            
            # Project each polygon vertex onto the road segment
            cut_volume = 0.0
            cut_depths = []
            cut_area = 0.0
            total_area = 0.0
            
            # Create a grid of points within polygon
            min_x, min_y = poly_points[:, 0].min(), poly_points[:, 1].min()
            max_x, max_y = poly_points[:, 0].max(), poly_points[:, 1].max()
            
            # Create grid
            grid_spacing = 0.5  # meters
            x_grid = np.arange(min_x, max_x, grid_spacing)
            y_grid = np.arange(min_y, max_y, grid_spacing)
            
            if len(x_grid) == 0 or len(y_grid) == 0:
                x_grid = np.linspace(min_x, max_x, 20)
                y_grid = np.linspace(min_y, max_y, 20)
            
            X, Y = np.meshgrid(x_grid, y_grid)
            grid_points = np.vstack([X.ravel(), Y.ravel()]).T
            
            # Check which grid points are inside polygon
            inside_mask = self.points_inside_polygon(grid_points, poly_points[:, :2])
            inside_points = grid_points[inside_mask]
            
            if len(inside_points) == 0:
                return None
            
            # Get terrain elevations at these points
            terrain_elev = self.get_elevation_from_pointcloud(inside_points)
            
            # For each inside point, check if it's within road width and get reference elevation
            ref_elevations = np.zeros(len(inside_points))
            road_mask = np.zeros(len(inside_points), dtype=bool)
            
            for i, (x, y) in enumerate(inside_points):
                # Project point onto road segment
                v = np.array([x, y]) - segment_start[:2]
                t = np.dot(v, road_dir)
                t = max(0, min(road_length, t))  # Clamp to segment
                
                # Get projected point on road centerline
                proj_point = segment_start[:2] + t * road_dir
                
                # Calculate distance from road centerline
                dist_from_center = np.linalg.norm(np.array([x, y]) - proj_point)
                
                # Check if point is within road width
                if dist_from_center <= half_width:
                    road_mask[i] = True
                    # Interpolate reference elevation
                    if road_length > 0:
                        ratio = t / road_length
                        ref_elev = segment_start[2] + ratio * (segment_end[2] - segment_start[2])
                    else:
                        ref_elev = segment_start[2]
                    ref_elevations[i] = ref_elev
            
            # Calculate cut volume only for points within road area
            if np.any(road_mask):
                road_terrain_elev = terrain_elev[road_mask]
                road_ref_elev = ref_elevations[road_mask]
                
                cut_depth = road_terrain_elev - road_ref_elev
                positive_cut = cut_depth > 0
                
                if np.any(positive_cut):
                    cell_area = grid_spacing ** 2
                    cut_volume = np.sum(cut_depth[positive_cut]) * cell_area
                    
                    stats = {
                        'cut_volume_m3': cut_volume,
                        'avg_cut_depth_m': np.mean(cut_depth[positive_cut]),
                        'max_cut_depth_m': np.max(cut_depth[positive_cut]),
                        'min_cut_depth_m': np.min(cut_depth[positive_cut]),
                        'polygon_area_m2': len(inside_points) * cell_area,
                        'road_area_m2': np.sum(road_mask) * cell_area,
                        'cut_area_m2': np.sum(positive_cut) * cell_area,
                        'road_width_m': road_width,
                        'grid_spacing_m': grid_spacing,
                        'num_grid_points': len(inside_points),
                        'num_road_points': np.sum(road_mask),
                        'reference_type': 'Construction/Road Baseline with Width',
                        'calculation_time': datetime.now().isoformat(),
                        'method': 'grid_based_with_width'
                    }
                else:
                    stats = {
                        'cut_volume_m3': 0.0,
                        'avg_cut_depth_m': 0.0,
                        'max_cut_depth_m': 0.0,
                        'min_cut_depth_m': 0.0,
                        'polygon_area_m2': len(inside_points) * cell_area,
                        'road_area_m2': np.sum(road_mask) * cell_area,
                        'cut_area_m2': 0.0,
                        'road_width_m': road_width,
                        'grid_spacing_m': grid_spacing,
                        'num_grid_points': len(inside_points),
                        'num_road_points': np.sum(road_mask),
                        'reference_type': 'Construction/Road Baseline with Width',
                        'calculation_time': datetime.now().isoformat(),
                        'method': 'grid_based_with_width',
                        'note': 'No cutting needed - terrain is below reference within road area'
                    }
            else:
                stats = {
                    'cut_volume_m3': 0.0,
                    'avg_cut_depth_m': 0.0,
                    'max_cut_depth_m': 0.0,
                    'min_cut_depth_m': 0.0,
                    'polygon_area_m2': len(inside_points) * cell_area,
                    'road_area_m2': 0.0,
                    'cut_area_m2': 0.0,
                    'road_width_m': road_width,
                    'grid_spacing_m': grid_spacing,
                    'num_grid_points': len(inside_points),
                    'num_road_points': 0,
                    'reference_type': 'Construction/Road Baseline with Width',
                    'calculation_time': datetime.now().isoformat(),
                    'method': 'grid_based_with_width',
                    'note': 'No points within road width area'
                }
            
            return stats
            
        except Exception as e:
            print(f"Error in cut volume calculation with width: {e}")
            import traceback
            traceback.print_exc()
            return None

    def load_reference_baseline_data(self, design_layer_path, reference_line_type):
        """Load reference baseline data from JSON file"""
        try:
            # Determine which file to load
            if reference_line_type == "Road Surface Line":
                filename = "road_surface_baseline.json"
            elif reference_line_type == "Construction Line":
                filename = "construction_baseline.json"
            else:
                return None
            
            json_path = os.path.join(design_layer_path, filename)
            
            if not os.path.exists(json_path):
                # Try alternative names
                alt_names = ["baseline.json", "reference_line.json", "line_data.json"]
                for alt_name in alt_names:
                    alt_path = os.path.join(design_layer_path, alt_name)
                    if os.path.exists(alt_path):
                        json_path = alt_path
                        break
                
                if not os.path.exists(json_path):
                    print(f"Reference file not found: {json_path}")
                    return None
            
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # Extract points from the JSON structure
            # Based on your construction_baseline.json structure
            if 'polylines' in data and len(data['polylines']) > 0:
                points = []
                for polyline in data['polylines']:
                    if 'points' in polyline:
                        for point_data in polyline['points']:
                            if 'world_coordinates' in point_data:
                                # Convert to [x, y, z]
                                points.append(point_data['world_coordinates'])
                return points
            elif 'points' in data:
                # Direct points array
                return data['points']
            elif 'world_coordinates' in data:
                # Single point
                return [data['world_coordinates']]
            else:
                print(f"Unknown JSON structure in {filename}")
                return None
                
        except Exception as e:
            print(f"Error loading reference baseline: {e}")
            return None

    def extract_reference_points_from_config(self, measurement_layer_name):
        """Extract reference points from saved measurement layer config"""
        try:
            layer_folder = os.path.join(
                self.WORKSHEETS_BASE_DIR,
                self.current_worksheet_name,
                "measurements",
                measurement_layer_name
            )
            
            config_file = os.path.join(layer_folder, "measurement_layer_config.txt")
            
            if not os.path.exists(config_file):
                return None
            
            with open(config_file, 'r', encoding='utf-8') as f:
                config = json.load(f)
            
            if not config.get('use_reference', False):
                return None
            
            design_path = config.get('reference_design_path')
            line_type = config.get('reference_line_type')
            
            if not design_path or not line_type:
                return None
            
            # Load the baseline data
            return self.load_reference_baseline_data(design_path, line_type)
            
        except Exception as e:
            print(f"Error extracting reference points: {e}")
            return None
    
    def points_inside_polygon(self, points, polygon_vertices):
        """Check if points are inside polygon"""
        import numpy as np
        
        # Simple ray casting algorithm
        x = points[:, 0]
        y = points[:, 1]
        n = len(polygon_vertices)
        inside = np.zeros(len(points), dtype=bool)
        
        for i in range(n):
            j = (i + 1) % n
            # Check if point is between y values of edge
            cond = ((polygon_vertices[i, 1] > y) != (polygon_vertices[j, 1] > y)) & \
                (x < (polygon_vertices[j, 0] - polygon_vertices[i, 0]) * (y - polygon_vertices[i, 1]) / 
                    (polygon_vertices[j, 1] - polygon_vertices[i, 1]) + polygon_vertices[i, 0])
            inside ^= cond
        
        return inside
    
    def get_elevation_from_pointcloud(self, points_xy):
        """Get elevation from point cloud at given XY coordinates"""
        import numpy as np
        from scipy.interpolate import griddata
        
        if hasattr(self, 'point_cloud') and self.point_cloud:
            # Get point cloud data
            cloud_points = np.asarray(self.point_cloud.points)
            
            if len(cloud_points) == 0:
                return np.zeros(len(points_xy))
            
            # Use existing polygon points if point cloud is not available
            cloud_xy = cloud_points[:, :2]
            cloud_z = cloud_points[:, 2]
            
            # Interpolate using nearest neighbor
            elevations = griddata(cloud_xy, cloud_z, points_xy, method='nearest', fill_value=np.nan)
            
            # Fill NaN values with minimum elevation
            if np.any(np.isnan(elevations)):
                min_elev = np.nanmin(elevations) if not np.all(np.isnan(elevations)) else 0
                elevations[np.isnan(elevations)] = min_elev
            
            return elevations
        
        # Fallback: return average of polygon points
        if hasattr(self, 'measurement_points'):
            poly_points = np.array(self.measurement_points)
            return np.full(len(points_xy), np.mean(poly_points[:, 2]))
        
        return np.zeros(len(points_xy))
    
    def process_cut_hill_volume(self):
        """Main function to calculate cut volume considering road width"""
        
        # Check if polygon is drawn
        if not hasattr(self, 'measurement_points') or len(self.measurement_points) < 3:
            QMessageBox.warning(self, "No Polygon", 
                            "Please draw a polygon first using the Polygon button.")
            return
        
        # Check if we have a measurement layer
        if not hasattr(self, 'current_measurement_layer'):
            QMessageBox.warning(self, "No Measurement Layer", 
                            "Please create a measurement layer first.")
            return
        
        # Load reference data
        reference_data = self.extract_reference_points_from_config(self.current_measurement_layer)
        
        if not reference_data:
            QMessageBox.warning(self, "No Reference Data",
                            "Could not load reference baseline data.")
            return
        
        # Get road width from reference configuration
        road_width = self.get_road_width_from_reference(self.current_measurement_layer)
        
        # Calculate cut volume with width consideration
        volume_result = self.calculate_cut_volume_with_width(
            self.measurement_points, 
            reference_data,
            road_width
        )
        
        if not volume_result:
            QMessageBox.warning(self, "Calculation Failed",
                            "Could not calculate cut volume.")
            return
        
        # Display results
        self.display_cut_volume_results(volume_result)
        
        # Extract and visualize cropped area
        self.extract_and_visualize_excavation_with_width(
            polygon_points=self.measurement_points,
            reference_data=reference_data,
            volume_result=volume_result,
            road_width=road_width
        )

    def get_road_width_from_reference(self, measurement_layer_name):
        """Get road width from measurement layer configuration"""
        try:
            layer_folder = os.path.join(
                self.WORKSHEETS_BASE_DIR,
                self.current_worksheet_name,
                "measurements",
                measurement_layer_name
            )
            
            config_file = os.path.join(layer_folder, "measurement_layer_config.txt")
            
            if not os.path.exists(config_file):
                return 10.0  # Default width
            
            with open(config_file, 'r', encoding='utf-8') as f:
                config = json.load(f)
            
            if not config.get('use_reference', False):
                return 10.0
            
            design_path = config.get('reference_design_path')
            line_type = config.get('reference_line_type')
            
            if not design_path or not line_type:
                return 10.0
            
            # Try to load the reference JSON to get width
            if line_type == "Construction Line":
                json_file = "construction_baseline.json"
            elif line_type == "Road Surface Line":
                json_file = "road_surface_baseline.json"
            else:
                return 10.0
            
            json_path = os.path.join(design_path, json_file)
            
            if os.path.exists(json_path):
                with open(json_path, 'r', encoding='utf-8') as f:
                    ref_data = json.load(f)
                
                # Extract width from JSON
                if 'width_meters' in ref_data:
                    return ref_data['width_meters']
            
            return 10.0  # Default if not found
            
        except Exception as e:
            print(f"Error getting road width: {e}")
            return 10.0
        
    def extract_and_visualize_excavation_with_width(self, polygon_points, reference_data, volume_result, road_width):
        """Visualize excavation considering road width"""
        
        try:
            import numpy as np
            
            # Convert inputs
            poly_array = np.array(polygon_points)
            
            if isinstance(reference_data[0], dict):
                ref_points = np.array([p['world_coordinates'] for p in reference_data 
                                    if 'world_coordinates' in p])
            else:
                ref_points = np.array(reference_data)
            
            if len(ref_points) < 2:
                QMessageBox.warning(self, "Insufficient Reference Data",
                                "Need at least 2 reference points.")
                return
            
            # Find the reference segment closest to polygon center
            center_x = np.mean(poly_array[:, 0])
            center_y = np.mean(poly_array[:, 1])
            
            distances = np.sqrt(
                (ref_points[:, 0] - center_x)**2 + 
                (ref_points[:, 1] - center_y)**2
            )
            closest_idx = np.argmin(distances)
            
            # Get the segment
            if closest_idx < len(ref_points) - 1:
                seg_start = ref_points[closest_idx]
                seg_end = ref_points[closest_idx + 1]
            else:
                seg_start = ref_points[closest_idx - 1]
                seg_end = ref_points[closest_idx]
            
            # Calculate road direction and perpendicular
            road_dir = seg_end[:2] - seg_start[:2]
            road_length = np.linalg.norm(road_dir)
            if road_length > 0:
                road_dir = road_dir / road_length
            
            perp_dir = np.array([-road_dir[1], road_dir[0]])
            half_width = road_width / 2
            
            # Create road boundaries
            left_start = seg_start[:2] - perp_dir * half_width
            left_end = seg_end[:2] - perp_dir * half_width
            right_start = seg_start[:2] + perp_dir * half_width
            right_end = seg_end[:2] + perp_dir * half_width
            
            # Create 3D points for boundaries (use interpolated elevations)
            left_boundary = []
            right_boundary = []
            road_surface_points = []
            
            # Create multiple points along the road segment
            num_segments = 10
            for i in range(num_segments + 1):
                t = i / num_segments
                # Interpolate along segment
                point_2d = seg_start[:2] + t * (seg_end[:2] - seg_start[:2])
                elev = seg_start[2] + t * (seg_end[2] - seg_start[2])
                
                # Left boundary
                left_point = np.array([point_2d[0] - perp_dir[0] * half_width,
                                    point_2d[1] - perp_dir[1] * half_width,
                                    elev])
                left_boundary.append(left_point)
                
                # Right boundary
                right_point = np.array([point_2d[0] + perp_dir[0] * half_width,
                                    point_2d[1] + perp_dir[1] * half_width,
                                    elev])
                right_boundary.append(right_point)
                
                # Road surface points (centerline)
                road_surface_points.append([point_2d[0], point_2d[1], elev])
            
            left_boundary = np.array(left_boundary)
            right_boundary = np.array(right_boundary)
            road_surface_points = np.array(road_surface_points)
            
            # Create road surface mesh points (triangulated surface)
            road_surface_mesh_points = []
            for i in range(num_segments):
                # Create two triangles for each segment to form a surface
                # Triangle 1: left[i], left[i+1], right[i]
                road_surface_mesh_points.extend([left_boundary[i], left_boundary[i+1], right_boundary[i]])
                # Triangle 2: right[i], left[i+1], right[i+1]
                road_surface_mesh_points.extend([right_boundary[i], left_boundary[i+1], right_boundary[i+1]])
            
            road_surface_mesh_points = np.array(road_surface_mesh_points)
            
            # Crop point cloud to polygon area
            cropped_cloud = self.crop_point_cloud_to_polygon(polygon_points)
            
            if cropped_cloud is None or len(cropped_cloud) == 0:
                # Use polygon vertices as fallback
                cropped_cloud = poly_array
            
            # Create excavation model
            excavation_model = self.create_excavation_model_with_road(
                polygon_points=poly_array,
                road_surface_points=road_surface_mesh_points,
                left_boundary=left_boundary,
                right_boundary=right_boundary,
                cropped_cloud=cropped_cloud,
                road_width=road_width,
                reference_segment=(seg_start, seg_end)
            )
            
            # Show in visualization
            self.show_excavation_viewer_with_road(
                cropped_cloud=cropped_cloud,
                excavation_model=excavation_model,
                polygon_points=poly_array,
                volume_result=volume_result,
                road_width=road_width
            )
            
        except Exception as e:
            QMessageBox.critical(self, "Visualization Error",
                            f"Error creating excavation model with width:\n{str(e)}")
            print(f"Error in width visualization: {e}")
            import traceback
            traceback.print_exc()

    def create_excavation_model_with_road(self, polygon_points, road_surface_points, left_boundary, 
                                        right_boundary, cropped_cloud, road_width, reference_segment=None):
        """Create 3D excavation model with road boundaries"""
        
        import numpy as np
        
        # Create vertical planes from polygon edges to road boundaries
        vertical_planes = []
        excavation_faces = []
        
        # For each polygon vertex, find the closest point on road boundaries
        for i, vertex in enumerate(polygon_points):
            # Find closest point on left boundary
            if len(left_boundary) > 0:
                left_distances = np.sqrt(
                    (left_boundary[:, 0] - vertex[0])**2 + 
                    (left_boundary[:, 1] - vertex[1])**2
                )
                left_closest_idx = np.argmin(left_distances)
                left_closest_point = left_boundary[left_closest_idx]
            else:
                left_closest_point = None
            
            # Find closest point on right boundary
            if len(right_boundary) > 0:
                right_distances = np.sqrt(
                    (right_boundary[:, 0] - vertex[0])**2 + 
                    (right_boundary[:, 1] - vertex[1])**2
                )
                right_closest_idx = np.argmin(right_distances)
                right_closest_point = right_boundary[right_closest_idx]
            else:
                right_closest_point = None
            
            # Use the closer boundary point
            if left_closest_point is not None and right_closest_point is not None:
                left_dist = np.linalg.norm(left_closest_point[:2] - vertex[:2])
                right_dist = np.linalg.norm(right_closest_point[:2] - vertex[:2])
                
                if left_dist < right_dist:
                    road_point = left_closest_point
                else:
                    road_point = right_closest_point
            elif left_closest_point is not None:
                road_point = left_closest_point
            elif right_closest_point is not None:
                road_point = right_closest_point
            else:
                # No road boundary points available
                continue
            
            # Create vertical line from road to terrain
            vertical_line = np.array([
                [road_point[0], road_point[1], road_point[2]],  # Road level
                [vertex[0], vertex[1], vertex[2]]               # Terrain level
            ])
            vertical_planes.append(vertical_line)
        
        # Create excavation faces (vertical walls)
        for i in range(len(polygon_points)):
            next_i = (i + 1) % len(polygon_points)
            
            # Create a vertical face for each polygon edge
            face_points = np.array([
                polygon_points[i],                          # Top vertex 1
                polygon_points[next_i],                     # Top vertex 2
                [polygon_points[next_i][0], polygon_points[next_i][1], 
                road_surface_points[0][2] if len(road_surface_points) > 0 else polygon_points[next_i][2] - 1],  # Bottom vertex 2
                [polygon_points[i][0], polygon_points[i][1], 
                road_surface_points[0][2] if len(road_surface_points) > 0 else polygon_points[i][2] - 1]       # Bottom vertex 1
            ])
            excavation_faces.append(face_points)
        
        return {
            'vertical_planes': vertical_planes,
            'excavation_faces': excavation_faces,
            'road_surface_points': road_surface_points,
            'left_boundary': left_boundary,
            'right_boundary': right_boundary,
            'polygon_vertices': polygon_points,
            'cropped_cloud': cropped_cloud,
            'road_width': road_width,
            'reference_segment': reference_segment
        }
    
    def display_cut_volume_results(self, result):
        """Display cut volume results with width information"""
        
        self.output_list.clear()
        
        # Header
        self.output_list.addItem("=" * 60)
        self.output_list.addItem("CUT HILL VOLUME CALCULATION WITH ROAD WIDTH")
        self.output_list.addItem("=" * 60)
        
        # Road info
        if 'road_width_m' in result:
            self.output_list.addItem(f"Road Width: {result['road_width_m']:.2f} m")
        
        # Basic info
        self.output_list.addItem(f"Reference Type: {result.get('reference_type', 'N/A')}")
        self.output_list.addItem(f"Calculation Method: {result.get('method', 'N/A')}")
        self.output_list.addItem("")
        
        # Area info
        self.output_list.addItem(f"Polygon Area: {result.get('polygon_area_m2', 0):.2f} m²")
        if 'road_area_m2' in result:
            self.output_list.addItem(f"Road Area within Polygon: {result.get('road_area_m2', 0):.2f} m²")
        self.output_list.addItem(f"Area Needing Cut: {result.get('cut_area_m2', 0):.2f} m²")
        self.output_list.addItem("")
        
        # Volume results
        self.output_list.addItem("VOLUME RESULTS:")
        self.output_list.addItem(f"  Total Cut Volume: {result.get('cut_volume_m3', 0):.2f} m³")
        self.output_list.addItem("")
        
        # Depth analysis
        self.output_list.addItem("DEPTH ANALYSIS:")
        self.output_list.addItem(f"  Average Cut Depth: {result.get('avg_cut_depth_m', 0):.2f} m")
        self.output_list.addItem(f"  Maximum Cut Depth: {result.get('max_cut_depth_m', 0):.2f} m")
        self.output_list.addItem(f"  Minimum Cut Depth: {result.get('min_cut_depth_m', 0):.2f} m")
        self.output_list.addItem("")
        
        # Grid info
        if 'grid_spacing_m' in result:
            self.output_list.addItem(f"Grid Resolution: {result['grid_spacing_m']} m")
            self.output_list.addItem(f"Grid Points: {result.get('num_grid_points', 0)}")
            if 'num_road_points' in result:
                self.output_list.addItem(f"Points within Road Area: {result.get('num_road_points', 0)}")
        
        # Notes
        if 'note' in result:
            self.output_list.addItem("")
            self.output_list.addItem(f"Note: {result['note']}")
        
        self.output_list.addItem("=" * 60)
        
        # Update message text
        volume = result.get('cut_volume_m3', 0)
        self.message_text.append(f"Cut Hill Calculation Complete: {volume:.2f} m³")
        if 'road_width_m' in result:
            self.message_text.append(f"Road Width Considered: {result['road_width_m']:.2f} m")

    def crop_point_cloud_to_polygon(self, polygon_points):
            """Extract point cloud data within the polygon"""
            
            import numpy as np
            
            if not hasattr(self, 'point_cloud') or self.point_cloud is None:
                print("No point cloud loaded")
                return None
            
            try:
                # Convert point cloud to numpy array
                all_points = np.asarray(self.point_cloud.points)
                
                if len(all_points) == 0:
                    return None
                
                # Convert polygon to numpy
                poly_array = np.array(polygon_points)
                
                # Get polygon bounds
                min_x, min_y = poly_array[:, 0].min(), poly_array[:, 1].min()
                max_x, max_y = poly_array[:, 0].max(), poly_array[:, 1].max()
                
                # First filter by bounding box (fast)
                bbox_mask = (all_points[:, 0] >= min_x) & (all_points[:, 0] <= max_x) & \
                            (all_points[:, 1] >= min_y) & (all_points[:, 1] <= max_y)
                
                bbox_points = all_points[bbox_mask]
                
                if len(bbox_points) == 0:
                    return None
                
                # Then filter by exact polygon (slower but accurate)
                inside_mask = self.points_inside_polygon(bbox_points[:, :2], poly_array[:, :2])
                cropped_points = bbox_points[inside_mask]
                
                print(f"Cropped {len(cropped_points)} points from polygon area")
                return cropped_points
                
            except Exception as e:
                print(f"Error cropping point cloud: {e}")
                return None
            
    def create_excavation_3d_model(self, polygon_points, reference_points, cropped_cloud):
        """Create 3D model of excavation with vertical planes"""
        try:
            # Convert inputs to numpy
            poly_array = np.array(polygon_points)
            
            # Convert reference points
            if isinstance(reference_points[0], dict):
                ref_array = np.array([p['world_coordinates'] for p in reference_points 
                                    if 'world_coordinates' in p])
            else:
                ref_array = np.array(reference_points)
            
            # Find the reference line segment that runs through/under the polygon
            # For simplicity, use the reference points closest to polygon center
            
            # Get polygon center
            center_x = np.mean(poly_array[:, 0])
            center_y = np.mean(poly_array[:, 1])
            
            # Find closest reference point to center
            distances = np.sqrt(
                (ref_array[:, 0] - center_x)**2 + 
                (ref_array[:, 1] - center_y)**2
            )
            closest_idx = np.argmin(distances)
            
            # Get reference elevation at this point
            ref_elevation = ref_array[closest_idx, 2]
            
            # Create vertical planes from reference line to polygon edges
            vertical_planes = []
            vertical_sections = []
            
            # Create vertical planes at polygon vertices
            for i, vertex in enumerate(poly_array):
                # Create vertical line from reference to terrain at this vertex
                vertical_line = np.array([
                    [vertex[0], vertex[1], ref_elevation],  # Bottom (reference)
                    [vertex[0], vertex[1], vertex[2]]       # Top (terrain)
                ])
                vertical_planes.append(vertical_line)
                
                # Create vertical section (plane) at this vertex
                if i < len(poly_array) - 1:
                    next_vertex = poly_array[i + 1]
                else:
                    next_vertex = poly_array[0]  # Close the polygon
                    
                # Create a vertical quadrilateral plane
                vertical_section = np.array([
                    [vertex[0], vertex[1], ref_elevation],          # Bottom left
                    [next_vertex[0], next_vertex[1], ref_elevation], # Bottom right
                    [next_vertex[0], next_vertex[1], next_vertex[2]], # Top right
                    [vertex[0], vertex[1], vertex[2]]               # Top left
                ])
                vertical_sections.append(vertical_section)
            
            # Create bottom plane (excavation base)
            # Project polygon vertices to reference elevation
            bottom_plane = poly_array.copy()
            bottom_plane[:, 2] = ref_elevation
            
            # Create top plane (original terrain)
            top_plane = poly_array
            
            return {
                'vertical_planes': vertical_planes,
                'vertical_sections': vertical_sections,
                'bottom_plane': bottom_plane,
                'top_plane': top_plane,
                'reference_elevation': ref_elevation,
                'polygon_vertices': poly_array,
                'cropped_cloud': cropped_cloud
            }
            
        except Exception as e:
            print(f"Error creating 3D model: {e}")
            return None
    
    def calculate_cut_volume_with_reference(self, polygon_points, reference_points):
        """Calculate cut volume with reference baseline"""
        try:
            # Convert inputs
            poly_array = np.array(polygon_points)
            
            if isinstance(reference_points[0], dict):
                ref_array = np.array([p['world_coordinates'] for p in reference_points 
                                    if 'world_coordinates' in p])
            else:
                ref_array = np.array(reference_points)
            
            if len(ref_array) == 0:
                return None
            
            # Get polygon bounds
            min_x, min_y = poly_array[:, 0].min(), poly_array[:, 1].min()
            max_x, max_y = poly_array[:, 0].max(), poly_array[:, 1].max()
            
            # Create grid (adaptive resolution)
            area = (max_x - min_x) * (max_y - min_y)
            if area < 100:
                grid_spacing = 0.25
            elif area < 1000:
                grid_spacing = 0.5
            else:
                grid_spacing = 1.0
            
            # Create grid
            x_grid = np.arange(min_x, max_x, grid_spacing)
            y_grid = np.arange(min_y, max_y, grid_spacing)
            
            if len(x_grid) == 0 or len(y_grid) == 0:
                x_grid = np.linspace(min_x, max_x, 20)
                y_grid = np.linspace(min_y, max_y, 20)
            
            X, Y = np.meshgrid(x_grid, y_grid)
            grid_xy = np.vstack([X.ravel(), Y.ravel()]).T
            
            # Check inside polygon
            inside_mask = self.points_inside_polygon(grid_xy, poly_array[:, :2])
            inside_points = grid_xy[inside_mask]
            
            if len(inside_points) == 0:
                return None
            
            # Calculate elevations
            terrain_elev = self.get_elevation_from_pointcloud(inside_points)
            
            # Get reference elevations (nearest neighbor)
            from scipy.spatial import distance
            distances = distance.cdist(inside_points, ref_array[:, :2], 'euclidean')
            nearest_idx = np.argmin(distances, axis=1)
            ref_elev = ref_array[nearest_idx, 2]
            
            # Calculate cut volume
            cut_depth = terrain_elev - ref_elev
            positive_mask = cut_depth > 0
            
            cell_area = grid_spacing ** 2
            cut_volume = np.sum(cut_depth[positive_mask]) * cell_area
            
            # Calculate statistics
            if np.any(positive_mask):
                positive_depths = cut_depth[positive_mask]
                stats = {
                    'cut_volume_m3': cut_volume,
                    'avg_cut_depth_m': np.mean(positive_depths),
                    'max_cut_depth_m': np.max(positive_depths),
                    'min_cut_depth_m': np.min(positive_depths),
                    'polygon_area_m2': len(inside_points) * cell_area,
                    'cut_area_m2': len(positive_depths) * cell_area,
                    'grid_spacing_m': grid_spacing,
                    'num_grid_points': len(inside_points),
                    'reference_type': 'Construction/ Road Baseline',
                    'calculation_time': datetime.now().isoformat(),
                    'method': 'grid_based'
                }
            else:
                stats = {
                    'cut_volume_m3': 0.0,
                    'avg_cut_depth_m': 0.0,
                    'max_cut_depth_m': 0.0,
                    'min_cut_depth_m': 0.0,
                    'polygon_area_m2': len(inside_points) * cell_area,
                    'cut_area_m2': 0.0,
                    'grid_spacing_m': grid_spacing,
                    'num_grid_points': len(inside_points),
                    'reference_type': 'Construction/ Road Baseline',
                    'calculation_time': datetime.now().isoformat(),
                    'method': 'grid_based',
                    'note': 'No cutting needed - terrain is below reference'
                }
            
            return stats
            
        except Exception as e:
            print(f"Volume calculation error: {e}")
            return None

    def show_simple_excavation_view_with_road(self, polygon_points, volume_result, road_width):
        """Fallback simple visualization for road width case"""
        
        try:
            import numpy as np
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D
            
            fig = plt.figure(figsize=(14, 10))
            
            # 1. 3D View
            ax1 = fig.add_subplot(221, projection='3d')
            
            # Plot polygon points
            poly_array = np.array(polygon_points)
            ax1.scatter(poly_array[:, 0], poly_array[:, 1], poly_array[:, 2], 
                    c='green', label='Terrain Points', alpha=0.6)
            
            # Plot polygon boundary
            closed_poly = np.vstack([poly_array, poly_array[0]])
            ax1.plot(closed_poly[:, 0], closed_poly[:, 1], closed_poly[:, 2], 
                    'b-', linewidth=2, label='Polygon')
            
            ax1.set_title(f'3D Excavation View (Road Width: {road_width:.1f}m)')
            ax1.set_xlabel('X')
            ax1.set_ylabel('Y')
            ax1.set_zlabel('Z')
            ax1.legend()
            
            # 2. Top View (XY Plane) with road width visualization
            ax2 = fig.add_subplot(222)
            ax2.scatter(poly_array[:, 0], poly_array[:, 1], c='green', alpha=0.6)
            ax2.plot(closed_poly[:, 0], closed_poly[:, 1], 'b-', linewidth=2)
            
            # Draw a simplified road width
            # Find polygon center
            center_x = np.mean(poly_array[:, 0])
            center_y = np.mean(poly_array[:, 1])
            
            # Draw road width rectangle (simplified)
            road_length = np.max(poly_array[:, 0]) - np.min(poly_array[:, 0])
            rect_x = [center_x - road_length/2, center_x + road_length/2, 
                    center_x + road_length/2, center_x - road_length/2, center_x - road_length/2]
            rect_y = [center_y - road_width/2, center_y - road_width/2,
                    center_y + road_width/2, center_y + road_width/2, center_y - road_width/2]
            ax2.plot(rect_x, rect_y, 'r-', linewidth=2, label=f'Road Width: {road_width:.1f}m')
            
            ax2.set_title('Top View with Road Width')
            ax2.set_xlabel('X')
            ax2.set_ylabel('Y')
            ax2.grid(True)
            ax2.legend()
            ax2.axis('equal')
            
            # 3. Side View (XZ Plane)
            ax3 = fig.add_subplot(223)
            ax3.scatter(poly_array[:, 0], poly_array[:, 2], c='green', alpha=0.6)
            ax3.set_title('Side View (XZ Plane)')
            ax3.set_xlabel('X')
            ax3.set_ylabel('Elevation (Z)')
            ax3.grid(True)
            
            # 4. Text Info
            ax4 = fig.add_subplot(224)
            ax4.axis('off')
            
            volume = volume_result.get('cut_volume_m3', 0)
            area = volume_result.get('polygon_area_m2', 0)
            road_area = volume_result.get('road_area_m2', 0)
            avg_depth = volume_result.get('avg_cut_depth_m', 0)
            max_depth = volume_result.get('max_cut_depth_m', 0)
            
            info_text = f"""
            EXCAVATION SUMMARY WITH ROAD WIDTH
            
            Road Width: {road_width:.2f} m
            Cut Volume: {volume:.2f} m³
            Polygon Area: {area:.2f} m²
            Road Area: {road_area:.2f} m²
            
            Depth Analysis:
            • Average Depth: {avg_depth:.2f} m
            • Maximum Depth: {max_depth:.2f} m
            
            Calculation Method: {volume_result.get('method', 'N/A')}
            
            For full 3D visualization with road boundaries,
            install: pip install pyvista pyvistaqt
            """
            
            ax4.text(0, 0.9, info_text, fontsize=11, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
            
            plt.tight_layout()
            plt.show()
            
        except Exception as e:
            print(f"Simple visualization error: {e}")
            # Show results in text only
            self.display_cut_volume_results(volume_result)

    def show_excavation_viewer_with_road(self, cropped_cloud, excavation_model, polygon_points, volume_result, road_width):
        """Show excavation model with road visualization in a new 3D viewer window"""
        
        try:
            # Try to import PyVista for 3D visualization
            import pyvista as pv
            import numpy as np
            
            # Create a new Qt window for the excavation viewer
            from PyQt5.QtWidgets import QMainWindow, QVBoxLayout, QWidget, QLabel, QHBoxLayout, QPushButton
            from PyQt5.QtCore import Qt
            
            class ExcavationViewerWithRoad(QMainWindow):
                def __init__(self, cropped_cloud, excavation_model, volume_result, road_width):
                    super().__init__()
                    
                    self.cropped_cloud = cropped_cloud
                    self.excavation_model = excavation_model
                    self.volume_result = volume_result
                    self.road_width = road_width
                    
                    self.setWindowTitle("Excavation Model with Road Width - Cut Hill Visualization")
                    self.setGeometry(100, 100, 1400, 900)
                    
                    self.setup_ui()
                    self.setup_3d_view()
                    
                def setup_ui(self):
                    # Central widget
                    central_widget = QWidget()
                    self.setCentralWidget(central_widget)
                    
                    # Main layout
                    main_layout = QVBoxLayout(central_widget)
                    
                    # Info panel
                    info_panel = QWidget()
                    info_layout = QHBoxLayout(info_panel)
                    
                    # Volume info
                    volume = self.volume_result.get('cut_volume_m3', 0)
                    area = self.volume_result.get('polygon_area_m2', 0)
                    road_area = self.volume_result.get('road_area_m2', 0)
                    
                    info_text = f"""
                    <b>EXCAVATION MODEL WITH ROAD WIDTH</b><br>
                    Road Width: {self.road_width:.2f} m<br>
                    Cut Volume: {volume:.2f} m³<br>
                    Polygon Area: {area:.2f} m²<br>
                    Road Area: {road_area:.2f} m²<br>
                    Average Depth: {self.volume_result.get('avg_cut_depth_m', 0):.2f} m<br>
                    Max Depth: {self.volume_result.get('max_cut_depth_m', 0):.2f} m
                    """
                    
                    info_label = QLabel(info_text)
                    info_label.setStyleSheet("""
                        QLabel {
                            font-size: 12px;
                            padding: 10px;
                            background-color: #f0f0f0;
                            border: 1px solid #ccc;
                            border-radius: 5px;
                        }
                    """)
                    info_layout.addWidget(info_label)
                    
                    # View controls
                    controls_text = """
                    <b>VIEW CONTROLS</b><br>
                    • Left Click + Drag: Rotate<br>
                    • Right Click + Drag: Pan<br>
                    • Scroll: Zoom<br>
                    • Press 'r': Reset View<br>
                    • Press 'w': Wireframe<br>
                    • Press 's': Surface<br>
                    • Press '1': Show Road Surface<br>
                    • Press '2': Show Road Boundaries<br>
                    • Press '3': Show Terrain Points
                    """
                    
                    controls_label = QLabel(controls_text)
                    controls_label.setStyleSheet("""
                        QLabel {
                            font-size: 11px;
                            padding: 10px;
                            background-color: #e8f4f8;
                            border: 1px solid #b0d0e0;
                            border-radius: 5px;
                        }
                    """)
                    info_layout.addWidget(controls_label)
                    
                    # Add buttons for toggling visibility
                    button_layout = QHBoxLayout()
                    
                    self.btn_road_surface = QPushButton("Toggle Road Surface")
                    self.btn_road_surface.setCheckable(True)
                    self.btn_road_surface.setChecked(True)
                    self.btn_road_surface.clicked.connect(self.toggle_road_surface)
                    
                    self.btn_road_boundaries = QPushButton("Toggle Road Boundaries")
                    self.btn_road_boundaries.setCheckable(True)
                    self.btn_road_boundaries.setChecked(True)
                    self.btn_road_boundaries.clicked.connect(self.toggle_road_boundaries)
                    
                    self.btn_terrain = QPushButton("Toggle Terrain")
                    self.btn_terrain.setCheckable(True)
                    self.btn_terrain.setChecked(True)
                    self.btn_terrain.clicked.connect(self.toggle_terrain)
                    
                    button_layout.addWidget(self.btn_road_surface)
                    button_layout.addWidget(self.btn_road_boundaries)
                    button_layout.addWidget(self.btn_terrain)
                    
                    info_layout.addLayout(button_layout)
                    
                    main_layout.addWidget(info_panel)
                    
                    # Create PyVista Qt widget
                    try:
                        from pyvistaqt import QtInteractor
                        self.plotter = QtInteractor(central_widget)
                    except ImportError:
                        # Fallback: use regular plotter
                        self.plotter = pv.Plotter()
                        self.plotter.show(auto_close=False)
                        return
                    
                    main_layout.addWidget(self.plotter.interactor)
                    
                    # Store references to meshes for toggling
                    self.road_surface_mesh = None
                    self.road_boundary_meshes = []
                    self.terrain_mesh = None
                    self.vertical_plane_meshes = []
                    
                def toggle_road_surface(self):
                    if self.road_surface_mesh:
                        self.road_surface_mesh.SetVisibility(not self.road_surface_mesh.GetVisibility())
                        self.plotter.render()
                        
                def toggle_road_boundaries(self):
                    for mesh in self.road_boundary_meshes:
                        mesh.SetVisibility(not mesh.GetVisibility())
                    self.plotter.render()
                        
                def toggle_terrain(self):
                    if self.terrain_mesh:
                        self.terrain_mesh.SetVisibility(not self.terrain_mesh.GetVisibility())
                    self.plotter.render()
                    
                def setup_3d_view(self):
                    """Setup 3D visualization with road width"""
                    
                    # Clear previous plots
                    self.plotter.clear()
                    
                    # 1. Show cropped point cloud (original terrain)
                    if self.cropped_cloud is not None and len(self.cropped_cloud) > 0:
                        points_poly = pv.PolyData(self.cropped_cloud)
                        self.terrain_mesh = self.plotter.add_points(
                            points_poly, 
                            color='green', 
                            point_size=3, 
                            opacity=0.7,
                            label='Terrain Points'
                        )
                    
                    # 2. Show polygon boundary
                    poly_vertices = self.excavation_model['polygon_vertices']
                    if len(poly_vertices) > 0:
                        # Close the polygon
                        closed_poly = np.vstack([poly_vertices, poly_vertices[0]])
                        poly_line = pv.lines_from_points(closed_poly)
                        self.plotter.add_mesh(
                            poly_line, 
                            color='blue', 
                            line_width=4,
                            label='Polygon Boundary'
                        )
                    
                    # 3. Show road surface (if available)
                    if 'road_surface_points' in self.excavation_model:
                        road_points = self.excavation_model['road_surface_points']
                        if len(road_points) > 0:
                            # Create a mesh for the road surface
                            try:
                                # Create a polygonal mesh from road points
                                road_mesh = pv.PolyData(road_points)
                                # Try to create a surface
                                try:
                                    road_surface = road_mesh.delaunay_2d()
                                    self.road_surface_mesh = self.plotter.add_mesh(
                                        road_surface, 
                                        color='gray', 
                                        opacity=0.6, 
                                        label='Road Surface'
                                    )
                                except:
                                    # Fallback: show as points
                                    self.road_surface_mesh = self.plotter.add_points(
                                        road_points, 
                                        color='gray', 
                                        point_size=5,
                                        label='Road Surface Points'
                                    )
                            except Exception as e:
                                print(f"Error creating road surface mesh: {e}")
                    
                    # 4. Show road boundaries
                    if 'left_boundary' in self.excavation_model and 'right_boundary' in self.excavation_model:
                        left_boundary = self.excavation_model['left_boundary']
                        right_boundary = self.excavation_model['right_boundary']
                        
                        # Show left boundary
                        if len(left_boundary) > 0:
                            left_line = pv.lines_from_points(left_boundary)
                            left_mesh = self.plotter.add_mesh(
                                left_line, 
                                color='red', 
                                line_width=3,
                                label='Left Road Boundary'
                            )
                            self.road_boundary_meshes.append(left_mesh)
                        
                        # Show right boundary
                        if len(right_boundary) > 0:
                            right_line = pv.lines_from_points(right_boundary)
                            right_mesh = self.plotter.add_mesh(
                                right_line, 
                                color='red', 
                                line_width=3,
                                label='Right Road Boundary'
                            )
                            self.road_boundary_meshes.append(right_mesh)
                    
                    # 5. Show vertical planes (excavation walls)
                    if 'vertical_planes' in self.excavation_model:
                        for i, plane in enumerate(self.excavation_model['vertical_planes']):
                            if len(plane) >= 2:
                                try:
                                    # Create a line for the vertical plane
                                    line = pv.Line(plane[0], plane[1])
                                    plane_mesh = self.plotter.add_mesh(
                                        line, 
                                        color='orange',
                                        line_width=2,
                                        label='Vertical Cut' if i == 0 else ""
                                    )
                                    self.vertical_plane_meshes.append(plane_mesh)
                                except:
                                    pass
                    
                    # 6. Show excavation faces
                    if 'excavation_faces' in self.excavation_model:
                        for i, face in enumerate(self.excavation_model['excavation_faces']):
                            if len(face) >= 3:
                                try:
                                    # Create a face for the excavation
                                    faces = np.hstack([[len(face)] + list(range(len(face)))])
                                    face_mesh = pv.PolyData(face, faces)
                                    self.plotter.add_mesh(
                                        face_mesh, 
                                        color='yellow',
                                        opacity=0.3,
                                        label='Excavation Face' if i == 0 else ""
                                    )
                                except:
                                    pass
                    
                    # Add legend and configure view
                    self.plotter.add_legend()
                    self.plotter.show_grid()
                    self.plotter.enable_terrain_style()
                    
                    # Set camera position for better view
                    self.plotter.view_isometric()
                    
                    # Add title
                    volume = self.volume_result.get('cut_volume_m3', 0)
                    title_text = f"Cut Volume: {volume:.2f} m³ | Road Width: {self.road_width:.2f} m"
                    self.plotter.add_text(title_text, position='upper_left', font_size=14)
            
            # Create and show the excavation viewer window
            self.excavation_viewer = ExcavationViewerWithRoad(cropped_cloud, excavation_model, volume_result, road_width)
            self.excavation_viewer.show()
            
        except ImportError as e:
            print(f"PyVista or PyVistaQt not installed: {e}")
            # Fallback to simple visualization
            self.show_simple_excavation_view_with_road(polygon_points, volume_result, road_width)
            
        except Exception as e:
            print(f"Error showing excavation viewer with road: {e}")
            import traceback
            traceback.print_exc()



# =======================================================================================================================================
#=================== Approach road angle dialog handling =================== 
    def on_elivation_angle_button_clicked(self):
        
        dialog = ElevationangleDialog(self)
        
        if dialog.exec_() != QDialog.Accepted:
            return

        # Extract full config from dialog
        config = dialog.get_configuration()

        # Validate angle
        if config['angle'] <= 0:
            QMessageBox.warning(self, "Invalid Angle", "Please enter an angle greater than 0.")
            return
        
# --------------------------------------------------------
        # NEW: convert chainage → real 3D point (the actual clicked surface point)
        # Find the surface point that has this chainage
        # found = False
        # for line_type in ['surface']:
        #     for polyline in reversed(self.line_types[line_type]['polylines']):  # newest first
        #         for dist, rel_z in reversed(polyline):
        #             if abs(dist - last_x) < 0.5:  # small tolerance
        #                 t = dist / self.total_distance
        #                 dir_vec = self.zero_end_point - self.zero_start_point
        #                 pos = self.zero_start_point + t * dir_vec
        #                 pos[2] = self.zero_start_z + rel_z
        #                 self.curve_start_point_3d = pos.copy()
        #                 found = True
        #                 break
        #         if found:
        #             break
        #     if found:
        #         break

        # if not found:
        #     # fallback – just use zero line height
        #     t = last_x / self.total_distance
        #     dir_vec = self.zero_end_point - self.zero_start_point
        #     pos = self.zero_start_point + t * dir_vec
        #     pos[2] = self.zero_start_z
        #     self.curve_start_point_3d = pos.copy()
       

        # # Log in message area
        # chainage = self.get_chainage_label(last_x)
        # self.message_text.append(f"Approach road started: '{display_text}' at chainage {chainage}")
        self.message_text.append("Click any angle label to edit it individually.")

        self.canvas.draw_idle()
