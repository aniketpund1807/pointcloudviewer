# main.py
import os
import numpy as np
import open3d as o3d
import time
import json

from PyQt5.QtWidgets import (
    QVBoxLayout, QHBoxLayout, QLabel, QWidget, QPushButton, QFileDialog, QMessageBox, QDialog, QApplication,
    QCheckBox
)
from PyQt5.QtCore import Qt, QByteArray, QSize, QRectF, QTimer, QEvent
from PyQt5.QtGui import QPixmap, QPainter, QIcon
from PyQt5.QtSvg import QSvgRenderer

import vtk
from vtkmodules.vtkFiltersSources import vtkSphereSource, vtkLineSource
from vtkmodules.vtkRenderingCore import vtkActor, vtkPolyDataMapper
from vtkmodules.vtkInteractionStyle import vtkInteractorStyleTrackballCamera
from vtkmodules.vtkRenderingCore import (vtkActor, vtkPolyDataMapper)

from vtkmodules.vtkFiltersSources import vtkPlaneSource


from datetime import datetime
from math import sqrt, degrees

from utils import find_best_fitting_plane
from application_ui import ApplicationUI
from dialogs import (ConstructionConfigDialog, CurveDialog, ZeroLineDialog, MaterialLineDialog, MeasurementDialog,
                    DesignNewDialog, WorksheetNewDialog, HelpDialog, ConstructionNewDialog, CreateProjectDialog, ExistingWorksheetDialog,
                    RoadPlaneWidthDialog, MaterialSegmentDialog, NewMaterialLineDialog)

# ===================================================================================================================================================================   
#                                                                   ** CLASS POINTCLOUDVIEWER **
# ===================================================================================================================================================================
class PointCloudViewer(ApplicationUI):
    def __init__(self, username=None, user_id=None, user_full_name=None):  # Add user_full_name parameter
        super().__init__()
        self.current_user = username or "guest"  # Store logged-in user
        self.current_user_id = user_id or "guest"  # Store logged-in user's ID
        self.current_user_full_name = user_full_name or username or "Guest"  # Store full name for header

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

        self.curved_alignment = None  # Cache for curved path: list of (ch, pos_3d, dir_3d)
        
        self.road_plane_actors = []      # Stores the two side planes (left + right)
        self.road_plane_center_actor = None   # Optional: centre line for reference

        self.slider_marker_actor = None   # The sphere actor that follows the slider
        self.slider_marker_radius = 0.45  # Size of the marker sphere (adjust if needed)

        # NEW: storage for 3D curve actors (so we can clear them later)
        self.curve_3d_actors = []          # list of vtkActor for the yellow arcs
        self.curve_start_point_3d = None   # world coordinate of the point where curve started
        self.curve_start_chainage = None   # distance along zero line at start
        # NEW: 3D curve support
        self.surface_line_3d_points = []

        self.zero_interval = None  # Will be set when zero line is defined

        self.bottom_button_layout = None   # Add for the baseline name show in bottom section

        self.material_line_widgets = []

        self.material_drawing_points = []          # current points being drawn
        self.current_material_line_artist = None   # temporary line preview
        self.material_polylines = {}               # dict: material_idx -> list of finished polylines # Current session labels for material

        self.material_polylines_artists = {}  # {material_idx: [artist1, artist2, ...]}
        self.current_material_line_artist = None  # Only for temporary preview
        self.material_drawing_points = []

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
        
        # Define worksheet base directory - now points to the user's folder
        # Worksheets are created directly inside the user's folder (e.g., E:\3D_Tool\user\UHA-00001\Worksheet - 1)
        self.USER_BASE_DIR = r"E:\3D_Tool\user"
        self.WORKSHEETS_BASE_DIR = os.path.join(self.USER_BASE_DIR, self.current_user_id)
        os.makedirs(self.WORKSHEETS_BASE_DIR, exist_ok=True)

        # === ADD THIS: Projects base directory ===
        self.PROJECTS_BASE_DIR = r"E:\3D_Tool\projects"
        os.makedirs(self.PROJECTS_BASE_DIR, exist_ok=True)
        
        # Initialize specific attributes that need different values
        self.start_point = np.array([387211.43846649484, 2061092.3144329898, 598.9991744523196])
        self.end_point = np.array([387219.37847222225, 2060516.8861111111, 612.2502197265625])
        self.total_distance = np.sqrt((self.end_point[0] - self.start_point[0])**2 + (self.end_point[1] - self.start_point[1])**2)
        self.original_total_distance = self.total_distance

        # Ensure canvas can receive key events
        self.canvas.setFocusPolicy(Qt.StrongFocus)
        self.canvas.setFocus()  # Optional: give focus automatically

        # === MATERIAL LINE DRAWING STATE (NEW WORKFLOW) ===
        self.material_drawing_active = False                    # True only when Start is pressed
        self.material_drawing_points = []                       # Points currently being drawn
        self.current_material_line_artist = None                # Dashed preview line
        self.material_segment_labels = {}                       # {mat_idx: [label1, label2, ...]}
        self.active_material_index = None                       # Currently selected material line index
        self.material_polylines_artists = {}                    # Permanent saved lines {idx: [line1, line2]}

        # ────────────────────────────────────────────────────────────────
        #  IMPORTANT: Initialize storage dictionaries here
        # ────────────────────────────────────────────────────────────────
        self.material_fill_patches   = {}          # {material_idx: [{'bg': patch, 'hatch': patch}, ...]}
        self.material_3d_actors      = {}          # {material_idx: [vtkActor, ...]}

        # If you use these elsewhere, initialize them too
        self.material_segments       = []          # optional – list of segment dicts
        self.material_polylines      = {}
        
        self.baseline_widths = {}  # Add this in your __init__
        # Connect signals
        self.connect_signals()

        # NEW: Connect label click event AFTER everything is initialized
        self.label_pick_id = None  # Initialize the attribute
        
        # Add this method call to set up the label click handler
        QTimer.singleShot(100, self.setup_label_click_handler)  # Small delay to ensure canvas is ready

# =====================================================================================================================================================================
#                                                               ** BACKEND FUNCTIONALITY METHODS **
# =====================================================================================================================================================================

    #================================================ Function to setup label click handler after initialization ======================================================
    def setup_label_click_handler(self):
        """Set up the label click event handler after canvas is fully initialized"""
        if self.canvas:
            self.label_pick_id = self.canvas.mpl_connect('pick_event', self.on_label_click)  


# ============================================================== Function to handle label clicks ======================================================================
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

# ================================================================= Function to scroll graph with slider =============================================================
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
        
# ================================================================= Function to handle hover events for points
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
    
# ============================================================== Function to set measurement type ======================================================================
    def set_measurement_type(self, mtype):
        """Helper to switch measurement type"""
        self.current_measurement = mtype
        self.measurement_points = []
        self.message_text.append(f"Switched to {mtype.replace('_', ' ').title()}")

# =======================================================================================================================================
    def reset_measurement_tools(self):
        """Clear all measurement buttons and states"""
        self.line_button.setVisible(False)
        self.polygon_button.setVisible(False)
        self.stockpile_polygon_button.setVisible(False)
        self.complete_polygon_button.setVisible(False)
        self.presized_button.setVisible(False)
        self.metrics_group.setVisible(False)

# ============================================================= Function to handle global click events =================================================================
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
    
# ============================================================= Function to handle dropdown choice ====================================================================
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
            BASE_PROJECTS_DIR = r"E:\3D_Tool\projects"
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
        initial_layer_name = data["initial_layer_name"].strip()
        point_cloud_file = data.get("point_cloud_file")  # Full path or None

        # Determine point cloud filename for config
        pc_filename = os.path.basename(point_cloud_file) if point_cloud_file else "None"

        # Determine reference type and line
        ref_type = category if category in ["Road", "Bridge", "Other"] else "None"
        ref_line = None
        if category in ["Road", "Bridge"]:
            available_lines = ["Road_Layer_1", "Road_Layer_2", "Road_Layer_3"] if category == "Road" \
                              else ["Bridge_Layer_1", "Bridge_Layer_2", "Bridge_Layer_3"]
            ref_line = available_lines[0] if available_lines else None

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

        # Create designs or measurements folder
        if dimension == "2D":
            base_subfolder = os.path.join(worksheet_folder, "designs")
            self.message_text.append("   → Creating 'designs' folder for 2D Design mode")
        else:
            base_subfolder = os.path.join(worksheet_folder, "measurements")
            self.message_text.append("   → Creating 'measurements' folder for 3D Measurement mode")

        os.makedirs(base_subfolder, exist_ok=True)

        # Create initial layer folder if name provided
        layer_folder = None
        if initial_layer_name:
            layer_folder = os.path.join(base_subfolder, initial_layer_name)
            os.makedirs(layer_folder, exist_ok=True)
            self.message_text.append(f"   → Initial layer folder created: {initial_layer_name}")
            self.message_text.append(f"      Path: {layer_folder}")

            # === SAVE design_layer_config.txt for initial layer (only if 2D) ===
            if dimension == "2D":
                full_config = {
                    "layer_name": initial_layer_name,
                    "dimension": dimension,
                    "reference_type": ref_type,
                    "reference_line": ref_line or "None",
                    "project_name": project_name or "None",
                    "worksheet_name": worksheet_name,
                    "point_cloud_file": pc_filename,
                    "created_by": self.current_user,
                    "created_at": datetime.now().isoformat(),
                }
                config_file_path = os.path.join(layer_folder, "design_layer_config.txt")
                try:
                    with open(config_file_path, 'w', encoding='utf-8') as f:
                        json.dump(full_config, f, indent=4)
                    self.message_text.append("   → design_layer_config.txt saved for initial layer")
                except Exception as e:
                    self.message_text.append(f"   → Failed to save design_layer_config.txt: {str(e)}")
        else:
            self.message_text.append("   → No layer name entered — no initial layer folder created")

        # Save worksheet-level config
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

        # Update app state
        self.current_worksheet_name = worksheet_name
        self.current_project_name = project_name
        self.current_worksheet_data = config_data.copy()
        self.display_current_worksheet(config_data)

        if initial_layer_name:
            self.add_layer_to_panel(initial_layer_name, dimension)

        self.current_layer_name = initial_layer_name

        # UI visibility logic (same as before)
        if dimension == "3D":
            self.three_D_frame.setVisible(True)
            self.two_D_frame.setVisible(False)
        elif dimension == "2D":
            self.three_D_frame.setVisible(False)
            self.two_D_frame.setVisible(True)

        self.bottom_section.setVisible(category in ["Road", "Bridge"])

        # Hide all containers
        for container in [self.surface_container, self.construction_container, self.road_surface_container,
                          self.zero_container, self.deck_line_container, self.projection_container,
                          self.construction_dots_container, getattr(self, 'bridge_zero_container', None)]:
            if container:
                container.setVisible(False)

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

        # Buttons
        self.add_material_line_button.setVisible(False)
        self.preview_button.setVisible(True)
        self.threed_map_button.setVisible(True)
        self.save_button.setVisible(True)

        if not self.zero_line_set:
            self.zero_line.setChecked(True)
            if hasattr(self, 'bridge_zero_line'):
                self.bridge_zero_line.setChecked(True)

        self.canvas.draw()
        if hasattr(self, 'vtk_widget'):
            self.vtk_widget.GetRenderWindow().Render()

        # Auto-load point cloud
        if point_cloud_file and os.path.exists(point_cloud_file):
            self.message_text.append("Auto-loading selected point cloud...")
            success = self.load_point_cloud_from_path(point_cloud_file)
            self.message_text.append("Point cloud loaded successfully!" if success else "Failed to auto-load point cloud.")

        # Final message
        QMessageBox.information(self, "Success",
                                f"Worksheet '{worksheet_name}' created successfully!\n\n"
                                f"Dimension: {dimension} ({worksheet_type})\n"
                                f"Initial Layer: {initial_layer_name or '(none)'}\n"
                                f"Mode: {category or 'General'}\n"
                                f"Point Cloud: {pc_filename}\n\n"
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

        elif worksheet_type == "Design":
            self.mode_banner.setText("DESIGN MODE")
            self.mode_banner.setStyleSheet("""
            QLabel {
                    font-weight: bold;}
            """)

        else:
            self.mode_banner.setText("GENERAL MODE")

# =======================================================================================================================================
# # OPEN CREATE NEW DESIGN LAYER DIALOG
    def open_create_new_design_layer_dialog(self):
        """Open the Design New Layer dialog and save config to current worksheet's designs folder"""
        if not hasattr(self, 'current_worksheet_name') or not self.current_worksheet_name:
            QMessageBox.warning(self, "No Active Worksheet",
                                "Please create or open a worksheet first before creating a design layer.")
            return

        dialog = DesignNewDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            # === CLEAR EXISTING BASELINES BEFORE CREATING NEW LAYER ===
            # This allows users to create multiple design layers in one worksheet
            self.clear_current_design_layer()
            
            config = dialog.get_configuration()
            layer_name = config["layer_name"]
            dimension = config["dimension"]
            ref_type = config["reference_type"]
            ref_line = config["reference_line"]

            # Get current point cloud filename (from worksheet config or current state)
            pc_file = self.current_worksheet_data.get("point_cloud_file")
            pc_filename = os.path.basename(pc_file) if pc_file else "None"

            # Build path
            base_designs_path = os.path.join(self.WORKSHEETS_BASE_DIR, self.current_worksheet_name, "designs")
            layer_folder = os.path.join(base_designs_path, layer_name)
            os.makedirs(layer_folder, exist_ok=True)

            # === EXACT SAME CONFIG STRUCTURE ===
            full_config = {
                "layer_name": layer_name,
                "dimension": dimension,
                "reference_type": ref_type or "None",
                "reference_line": ref_line or "None",
                "project_name": getattr(self, 'current_project_name', 'None') or "None",
                "worksheet_name": self.current_worksheet_name,
                "point_cloud_file": pc_filename,
                "created_by": self.current_user,
                "created_at": datetime.now().isoformat(),
            }

            # Save config
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
                self.message_text.append(f" → Point Cloud: {pc_filename}")
                if ref_type:
                    self.message_text.append(f" → Reference Type: {ref_type} ({ref_line or 'No reference'})")

                self.add_layer_to_panel(layer_name, dimension)
                self.current_layer_name = layer_name

            except Exception as e:
                QMessageBox.critical(self, "Save Failed", f"Could not save design layer config:\n{str(e)}")
                return

            # UI updates (same as before)
            self.bottom_section.setVisible(True)
            for container in [self.surface_container, self.construction_container, self.road_surface_container,
                              self.zero_container, self.deck_line_container, self.projection_container,
                              self.construction_dots_container, getattr(self, 'bridge_zero_container', None)]:
                if container:
                    container.setVisible(False)

            if ref_type == "Road":
                self.surface_container.setVisible(True)
                self.construction_container.setVisible(True)
                self.road_surface_container.setVisible(True)
                self.zero_container.setVisible(True)
            elif ref_type == "Bridge":
                self.deck_line_container.setVisible(True)
                self.projection_container.setVisible(True)
                self.construction_dots_container.setVisible(True)
                if hasattr(self, 'bridge_zero_container'):
                    self.bridge_zero_container.setVisible(True)

            self.preview_button.setVisible(True)
            self.threed_map_button.setVisible(True)
            self.save_button.setVisible(True)

            if not self.zero_line_set:
                self.zero_line.setChecked(True)
                if hasattr(self, 'bridge_zero_line'):
                    self.bridge_zero_line.setChecked(True)

            self.canvas.draw()
            if hasattr(self, 'vtk_widget'):
                self.vtk_widget.GetRenderWindow().Render()
# =======================================================================================================================================
    def open_measurement_dialog(self):
        """Open the Measurement Configuration Dialog when New is clicked"""
        dialog = MeasurementDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            config = dialog.get_config()

            # Apply units
            idx = ["Meter", "Centimeter", "Millimeter"].index(config["units"].capitalize())
            self.metrics_combo.setCurrentIndex(idx)

            # Reset previous measurement state
            self.reset_measurement_tools()

            # Show relevant buttons
            self.metrics_group.setVisible(True)
            self.line_button.setVisible(config["measurement_type"] == "line")
            self.polygon_button.setVisible(config["measurement_type"] in ["polygon", "stockpile"])
            self.stockpile_polygon_button.setVisible(config["measurement_type"] == "stockpile")
            self.complete_polygon_button.setVisible(config["measurement_type"] in ["polygon", "stockpile"])
            self.presized_button.setVisible(config["presized_enabled"] and config["measurement_type"] == "line")

            # Set current measurement mode
            if config["measurement_type"] == "line":
                self.current_measurement = config["line_type"] + "_line"  # vertical_line, horizontal_line, general_line
                if config["line_type"] == "general":
                    self.current_measurement = "measurement_line"
            elif config["measurement_type"] == "polygon":
                self.current_measurement = "polygon"
            elif config["measurement_type"] == "stockpile":
                self.current_measurement = "polygon"
                self.message_text.append("Stockpile mode: Draw polygon → Complete → then click a height point")

            self.measurement_active = True
            self.plotting_active = False  # Disable graph drawing during measurement

            self.message_text.append(f"Started {config['measurement_type'].title()} measurement ({config['line_type'] if 'line_type' in config else ''})")
            self.message_text.append(f"Units set to: {config['units'].capitalize()}")

            # Connect actions
            self.vertical_line_action.triggered.connect(lambda: self.set_measurement_type('vertical_line'))
            self.horizontal_line_action.triggered.connect(lambda: self.set_measurement_type('horizontal_line'))
            self.measurement_line_action.triggered.connect(lambda: self.set_measurement_type('measurement_line'))
            self.polygon_button.clicked.connect(lambda: self.set_measurement_type('polygon'))
            if config["presized_enabled"]:
                self.presized_button.clicked.connect(self.handle_presized_button)

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

            # 6. Update UI – show bottom section
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

# ===================================================================================================================================
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
    def open_material_line_dialog(self):
        """Opens the appropriate dialog based on whether material_lines_config.txt exists.
        Now also creates and maintains material_lines_config.txt with full structure."""
        if not hasattr(self, 'current_worksheet_name') or not self.current_worksheet_name:
            QMessageBox.warning(self, "No Worksheet", "Please create or open a worksheet first.")
            return

        if not hasattr(self, 'construction_layer_info') or not self.construction_layer_info:
            QMessageBox.warning(self, "No Construction Layer", 
                                "Please create a construction layer first (Construction → New).")
            return

        current_construction_path = self.construction_layer_info.get('folder')
        if not current_construction_path or not os.path.exists(current_construction_path):
            QMessageBox.critical(self, "Path Error", 
                                f"Construction layer folder not found:\n{current_construction_path}")
            return

        self.current_construction_layer_path = current_construction_path

        material_lines_config_path = os.path.join(current_construction_path, "material_lines_config.txt")

        # Initialize or load config structure
        config_data = {
            "worksheet_name": self.current_worksheet_name or "Unknown Worksheet",
            "project_name": getattr(self, 'project_name', 'Project_1'),
            "created_by": getattr(self, 'current_user', 'admin123'),
            "created_at": "2026-01-05T14:53:39.382065",
            "material_line": []
        }

        if os.path.exists(material_lines_config_path):
            try:
                with open(material_lines_config_path, 'r', encoding='utf-8') as f:
                    loaded = json.load(f)
                config_data.update(loaded)
            except Exception as e:
                self.message_text.append(f"Error loading material_lines_config.txt: {str(e)}")

        if not os.path.exists(material_lines_config_path) or not config_data.get("material_line"):
            # === First material line – use old MaterialLineDialog ===
            dialog = MaterialLineDialog(
                construction_layer_path=current_construction_path,
                parent=self
            )

            if dialog.exec_() == QDialog.Accepted:
                material_data = dialog.get_material_data()
                folder_name = material_data['name'].strip()
                material_folder_full_path = os.path.join(current_construction_path, folder_name)

                config = {
                    'name': material_data['name'],
                    'folder_name': folder_name,
                    'material_type': material_data['material_type'],
                    'ref_layer': material_data['ref_layer'],
                    'visible': True,
                    'path': material_folder_full_path
                }

                self.material_configs.append(config)
                self.create_material_line_entry(config)

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
                        QPushButton:hover { background-color: #FF8C00; }
                        QPushButton:pressed { background-color: #FF7F00; }
                    """)

                self.message_text.append("Material Line Created Successfully!")
                self.message_text.append(f"Name: {config['name']}")
                self.message_text.append(f"Material Type: {config['material_type']}")
                self.message_text.append(f"Reference Baseline: {config['ref_layer']}")
                self.message_text.append("")

                # Save to config
                new_config_entry = {
                    "name": material_data['name'],
                    "material_type": material_data['material_type'],
                    "ref_layer": material_data['ref_layer']
                }
                config_data["material_line"].append(new_config_entry)

                try:
                    with open(material_lines_config_path, 'w', encoding='utf-8') as f:
                        json.dump(config_data, f, indent=4, ensure_ascii=False)
                    self.message_text.append("Created material_lines_config.txt")
                except Exception as e:
                    QMessageBox.critical(self, "Error", f"Failed to create material_lines_config.txt:\n{e}")

        else:
            # === Use NewMaterialLineDialog for subsequent materials ===
            try:
                material_lines = config_data.get("material_line", [])
                if not material_lines:
                    QMessageBox.information(self, "No Materials", "No material lines defined yet.")
                    return

                default_name = f"Material Line - {len(getattr(self.parent, 'material_configs', [])) + 1 if hasattr(self, 'parent') else len(self.material_configs) + 1}"

                material_config_for_dialog = {
                    'name': default_name,
                    'material_type': "",
                    'ref_layer': "construction",
                    '_display_ref_name': "Construction"
                }

            except Exception as e:
                self.message_text.append(f"Error preparing dialog: {str(e)}")
                material_config_for_dialog = {
                    'name': f"Material Line - {len(self.material_configs)+1}",
                    'material_type': "",
                    'ref_layer': "construction",
                    '_display_ref_name': "Construction"
                }

            dialog = NewMaterialLineDialog(material_config=material_config_for_dialog, parent=self)

            if dialog.exec_() == QDialog.Accepted:
                material_data = dialog.get_material_data()
                if material_data is None:
                    return

                folder_name = material_data['name'].strip()
                material_folder_full_path = os.path.join(current_construction_path, folder_name)

                config = {
                    'name': material_data['name'],
                    'folder_name': folder_name,
                    'material_type': material_data['material_type'],
                    'ref_layer': material_data['ref_layer'],  # str or list
                    'visible': True,
                    'path': material_folder_full_path
                }

                self.material_configs.append(config)
                self.create_material_line_entry(config)

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
                        QPushButton:hover { background-color: #FF8C00; }
                        QPushButton:pressed { background-color: #FF7F00; }
                    """)

                self.message_text.append("Material Line Created Successfully!")
                self.message_text.append(f"Name: {config['name']}")
                self.message_text.append(f"Material Type: {config['material_type']}")
                self.message_text.append(f"Reference Layer(s): {config['ref_layer']}")
                self.message_text.append("")

                # Update config file
                new_config_entry = {
                    "name": material_data['name'],
                    "material_type": material_data['material_type'],
                    "ref_layer": material_data['ref_layer']
                }
                config_data["material_line"].append(new_config_entry)

                try:
                    with open(material_lines_config_path, 'w', encoding='utf-8') as f:
                        json.dump(config_data, f, indent=4, ensure_ascii=False)
                except Exception as e:
                    QMessageBox.critical(self, "Error", f"Failed to update material_lines_config.txt:\n{e}")

                # === PROCESS SEGMENTS FROM DIALOG (Multiple references supported) ===
                material_idx = len(self.material_configs) - 1

                def parse_chainage(ch_str):
                    ch_str = str(ch_str).strip()
                    if '+' in ch_str:
                        parts = ch_str.split('+')
                        if len(parts) == 2:
                            try:
                                km = int(parts[0])
                                m = float(parts[1])
                                return km * 1000 + m
                            except:
                                pass
                    try:
                        return float(ch_str)
                    except:
                        return None

                # Load construction baseline once for elevation reference
                ref_xs, ref_ys = self._load_baseline_from_design_layer({'ref_layer': 'construction'})
                if ref_xs is None or len(ref_xs) == 0:
                    self.message_text.append("Error: Could not load construction baseline for elevation.")
                    return

                chainage_to_elev = dict(zip(ref_xs, ref_ys))

                for s_idx, seg in enumerate(material_data['segments'], 1):
                    from_str = seg['from_chainage']
                    to_str = seg['to_chainage']
                    from_m = parse_chainage(from_str)
                    to_m = parse_chainage(to_str)

                    if from_m is None or to_m is None or to_m <= from_m:
                        self.message_text.append(f"Invalid chainage in segment {s_idx}: {from_str} → {to_str}")
                        continue

                    # Safely parse thickness values
                    try:
                        init_val = seg.get('initial_thickness', 0)
                        if isinstance(init_val, str):
                            init_val = init_val.lower().replace('mm', '').strip()
                            thickness_m = float(init_val or 0) / 1000.0
                        else:
                            thickness_m = float(init_val or 0) / 1000.0
                    except:
                        thickness_m = 0.0

                    try:
                        after_val = seg.get('after_rolling', 0)
                        if isinstance(after_val, str):
                            after_val = after_val.lower().replace('mm', '').strip()
                            after_m = float(after_val or 0) / 1000.0
                        else:
                            after_m = float(after_val or 0) / 1000.0
                    except:
                        after_m = 0.0

                    try:
                        width_m = float(seg.get('width', 0))
                    except:
                        width_m = 0.0

                    # Get base elevation from construction at midpoint
                    mid_m = (from_m + to_m) / 2
                    base_elevation = chainage_to_elev.get(mid_m, 0.0)

                    top_elevation = base_elevation + thickness_m

                    polyline_points = [
                        {"chainage_m": from_m, "relative_elevation_m": top_elevation},
                        {"chainage_m": to_m, "relative_elevation_m": top_elevation}
                    ]

                    self.save_material_segment_to_json(
                        material_idx=material_idx,
                        config={
                            'from_chainage_m': from_m,
                            'to_chainage_m': to_m,
                            'material_thickness_m': thickness_m,
                            'width_m': width_m,
                            'after_rolling_thickness_m': after_m
                        },
                        from_m=from_m,
                        to_m=to_m,
                        point_number=s_idx,
                        polyline_points=polyline_points
                    )

                    # Draw top line
                    line = self.ax.plot(
                        [from_m, to_m],
                        [top_elevation, top_elevation],
                        color='orange',
                        linewidth=3,
                        linestyle='-',
                        marker='o',
                        markersize=6,
                        markerfacecolor='orange',
                        markeredgecolor='darkred',
                        alpha=0.9
                    )[0]

                    if material_idx not in self.material_polylines_artists:
                        self.material_polylines_artists[material_idx] = []
                    self.material_polylines_artists[material_idx].append(line)

                    # Label
                    mid_x = (from_m + to_m) / 2
                    mid_y = top_elevation + 0.25
                    label_text = f"M{material_idx+1}-{s_idx}"

                    annot = self.ax.annotate(
                        label_text, (mid_x, mid_y),
                        xytext=(0, 12), textcoords='offset points',
                        ha='center', va='bottom',
                        fontsize=9, fontweight='bold', color='white',
                        bbox=dict(boxstyle='round,pad=0.4', facecolor='orange', alpha=0.9, edgecolor='darkorange'),
                        picker=10
                    )

                    annot.point_data = {
                        'type': 'material_segment_label',
                        'material_index': material_idx,
                        'segment_number': s_idx,
                        'from_chainage_m': from_m,
                        'to_chainage_m': to_m,
                        'config': {}
                    }

                    if material_idx not in self.material_segment_labels:
                        self.material_segment_labels[material_idx] = []
                    self.material_segment_labels[material_idx].append(annot)

                    # Draw filling with hatching (only in this segment)
                    self.draw_material_filling(
                        from_chainage_m=from_m,
                        to_chainage_m=to_m,
                        thickness_m=thickness_m,
                        material_index=material_idx,
                        material_config=self.material_configs[material_idx],
                        color='#FF9800',
                        alpha=0.6,
                        width_m=width_m,
                        hatch_pattern='/',
                        base_elevation=base_elevation
                    )

                self.canvas.draw_idle()

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
                background-color: #FFA500; /* Orange background */
                border: 1px solid #FF8C00;
                border-radius: 5px;
                margin: 3px;
                padding: 5px;
            }
        """)

        layout = QHBoxLayout(container)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)

        # Checkbox for visibility/selection - START UNCHECKED for NEW items
        checkbox = QCheckBox()
        # Only use the provided 'visible' value when editing an existing entry
        # For new entries (edit_index is None), force it to False
        is_visible = material_data.get('visible', False) if edit_index is not None else False
        checkbox.setChecked(is_visible)

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

        # Get index for lambda (before append)
        current_index = len(self.material_items) if edit_index is None else edit_index

        # Connect checkbox to toggle visibility
        checkbox.stateChanged.connect(lambda state: self.toggle_material_line_visibility(current_index, state))

        # Manual trigger if already checked (important: stateChanged doesn't fire on init)
        if checkbox.isChecked():
            self.toggle_material_line_visibility(current_index, Qt.Checked)

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
        pencil_button.clicked.connect(lambda checked: self.edit_existing_material_line(current_index))

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
    def save_material_data(self):
        """Save all material lines including dot configurations to JSON file"""
        if not hasattr(self, 'material_configs') or not self.material_configs:
            self.message_text.append("No material lines to save.")
            return

        save_path = self.get_current_design_layer_path()
        if not save_path:
            return

        materials_data = []
        for i, config in enumerate(self.material_configs):
            data = config.copy()
            data['index'] = i

            # Add dots for this material
            material_dots = [d for d in self.material_dots if d['material_index'] == i]
            dots_info = []
            for dot in material_dots:
                dot_info = {
                    'point_number': dot['point_number'],
                    'chainage_m': dot['x'],
                    'elevation_m': dot['y'],
                    'config': dot['config']
                }
                if self.zero_start_km is not None:
                    interval_number = int(dot['x'] / self.zero_interval) if self.zero_interval > 0 else 0
                    interval_value = interval_number * self.zero_interval
                    chainage_label = f"{self.zero_start_km}+{interval_value:03d}"
                    if self.zero_interval > 0 and abs(dot['x'] - interval_value) > 0.01:
                        chainage_label += f" (approx {dot['x']:.2f}m)"
                    dot_info['chainage_label'] = chainage_label
                dots_info.append(dot_info)

            data['dots'] = dots_info
            data['num_dots'] = len(dots_info)
            materials_data.append(data)

        json_path = os.path.join(save_path, "material_lines.json")
        try:
            with open(json_path, 'w', encoding='utf-8') as f:
                json.dump(materials_data, f, indent=4, ensure_ascii=False)
            self.message_text.append(f"Material lines saved to: {json_path}")
        except Exception as e:
            self.message_text.append(f"Error saving material_lines.json: {str(e)}")


# ===========================================================================================================================================================
    def open_existing_worksheet(self):
        dialog = ExistingWorksheetDialog(self)
        if dialog.exec_() != QDialog.Accepted:
            return

        data = dialog.get_selected_data()
        if not data:
            QMessageBox.warning(self, "Invalid Selection", "No valid worksheet/layer selected.")
            return

        # === CLEAR PREVIOUS WORKSHEET DATA BEFORE LOADING NEW ONE ===
        # This clears all baselines, material lines, legends, annotations, 
        # and 3D visualizations from the previous worksheet
        if hasattr(self, 'current_worksheet_name') and self.current_worksheet_name:
            self.clear_current_design_layer()
            self.message_text.append("Previous worksheet data cleared.")

        config = data["worksheet_data"]
        worksheet_name = config.get("worksheet_name")
        layer_name = data["layer_name"]
        subfolder_type = data["subfolder_type"]  # "designs" or "construction"
        full_layer_path = data["full_layer_path"]

        # Worksheet root folder (one level above designs/construction)
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

        dimension = config.get("dimension", "2D")
        category = config.get("worksheet_category", "None")

        self.three_D_frame.setVisible(dimension == "3D")
        self.two_D_frame.setVisible(dimension == "2D")

        # Load layer config file
        config_filename = "Construction_Layer_config.txt" if subfolder_type == "construction" else "design_layer_config.txt"
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
            design_layer_path = os.path.normpath(design_layer_path)

        zero_loaded = False
        design_points_loaded = False
        loaded_baselines = {}

        # ===============================================================
        # DIFFERENT BEHAVIOR BASED ON SUBFOLDER TYPE
        # ===============================================================
        if subfolder_type == "designs":
            self.add_material_line_button.setVisible(False)
            loaded_baselines = self.load_all_baselines_from_layer(full_layer_path)
            design_points_loaded = self.load_json_files_to_3d_pointcloud(full_layer_path)

            # === RECREATE CURVE LABELS FROM SAVED ANGLES ===
            self.clear_curve_labels()
            recreated_curve_count = 0
            for ltype, baseline_data in loaded_baselines.items():
                for poly in baseline_data.get("polylines", []):
                    for pt in poly.get("points", []):
                        if "angle_deg" in pt:
                            chainage = pt["chainage_m"]
                            config_dict = {
                                'angle': pt["angle_deg"],
                                'inner_curve': pt.get("inner_curve", False),
                                'outer_curve': pt.get("outer_curve", False)
                            }
                            self.curve_labels.append({'chainage': chainage, 'config': config_dict})
                            self.add_curve_label_at_x(chainage, config_dict)
                            recreated_curve_count += 1

            if recreated_curve_count > 0:
                self.message_text.append(f"Recreated {recreated_curve_count} curve label(s) → curved road ready!")
            else:
                self.message_text.append("No curve angles found → straight road.")

            for ltype in loaded_baselines.keys():
                self.redraw_baseline_on_graph(ltype, style="solid")

            zero_loaded = self.load_zero_line_from_layer(full_layer_path)
            self.message_text.append(f"Design layer loaded: {len(loaded_baselines)} baselines (solid lines)")

        elif subfolder_type == "construction":
            # ... [your existing construction mode code unchanged] ...
            # (kept exactly as you had it — no changes here)
            referenced_baselines = layer_config.get("base_lines_reference", [])
            if isinstance(referenced_baselines, str):
                referenced_baselines = [referenced_baselines]

            dotted_lines_drawn = 0
            for baseline_filename in referenced_baselines:
                if not baseline_filename:
                    continue
                baseline_path = os.path.join(full_layer_path, baseline_filename)
                source = "construction layer"
                if not os.path.exists(baseline_path) and design_layer_path and os.path.exists(design_layer_path):
                    baseline_path = os.path.join(design_layer_path, baseline_filename)
                    source = "referenced design layer"

                if os.path.exists(baseline_path):
                    try:
                        with open(baseline_path, 'r', encoding='utf-8') as f:
                            data = json.load(f)
                        ltype = data.get("baseline_key", "construction")
                        color = data.get("color", "gray")
                        polylines_2d = []
                        for poly in data.get("polylines", []):
                            poly_2d = [(pt["chainage_m"], pt["relative_elevation_m"]) for pt in poly.get("points", [])]
                            if len(poly_2d) >= 2:
                                polylines_2d.append(poly_2d)
                        if ltype not in self.line_types:
                            self.line_types[ltype] = {'color': color, 'polylines': [], 'artists': []}
                        for artist in self.line_types[ltype]['artists']:
                            try:
                                artist.remove()
                            except:
                                pass
                        self.line_types[ltype]['artists'].clear()
                        self.line_types[ltype]['polylines'] = polylines_2d
                        all_x, all_y = [], []
                        for poly in polylines_2d:
                            xs, ys = zip(*poly)
                            all_x.extend(xs)
                            all_y.extend(ys)
                        if all_x:
                            artist, = self.ax.plot(all_x, all_y, color=color, linestyle=':', linewidth=3, alpha=0.8, zorder=5)
                            self.line_types[ltype]['artists'].append(artist)
                            dotted_lines_drawn += 1
                        self.message_text.append(f"Loaded reference baseline: {baseline_filename} ({source})")
                    except Exception as e:
                        self.message_text.append(f"Failed to load {baseline_filename}: {str(e)}")
                else:
                    self.message_text.append(f"Referenced baseline not found: {baseline_filename}")

            if design_layer_path and os.path.exists(design_layer_path):
                loaded_baselines = self.load_all_baselines_from_layer(design_layer_path)
                design_points_loaded = self.load_json_files_to_3d_pointcloud(design_layer_path)
                self.message_text.append(f"Loaded {len(loaded_baselines)} baselines for 3D planes from design layer")

            if design_layer_path and os.path.exists(design_layer_path):
                zero_loaded = self.load_zero_line_from_layer(design_layer_path)
            if not zero_loaded:
                zero_loaded = self.load_zero_line_from_layer(full_layer_path)

            self.current_construction_layer_path = full_layer_path

            config_path = os.path.join(full_layer_path, "material_lines_config.txt")
            if os.path.exists(config_path):
                try:
                    with open(config_path, 'r', encoding='utf-8') as f:
                        config_data = json.load(f)
                    material_lines = config_data.get("material_line", [])
                    self.material_configs = []
                    for mat in material_lines:
                        name = mat.get("name")
                        folder_name = name.strip()
                        mat_config = {
                            'name': name,
                            'folder_name': folder_name,
                            'material_type': mat.get("material_type"),
                            'ref_layer': mat.get("ref_layer"),
                            'visible': True,
                            'path': os.path.join(full_layer_path, folder_name)
                        }
                        self.material_configs.append(mat_config)
                        self.create_material_line_entry(mat_config)
                        if self.material_line_widgets:
                            material_widget = self.material_line_widgets[-1]
                            checkbox = material_widget.findChild(QCheckBox)
                            if checkbox:
                                checkbox.setChecked(True)
                    for idx in range(len(self.material_configs)):
                        self.load_and_draw_material_filling(idx)
                    self.message_text.append(f"Loaded {len(self.material_configs)} material lines")
                except Exception as e:
                    self.message_text.append(f"Error loading material config: {str(e)}")

        # ===============================================================
        # Generate 3D planes
        # ===============================================================
        if self.zero_line_set and loaded_baselines:
            #self.clear_baseline_planes()
            planes_generated = 0
            width_summary = []
            for ltype, baseline_data in loaded_baselines.items():
                width_m = baseline_data.get("width_meters")
                if width_m is None or width_m <= 0:
                    self.message_text.append(f"Warning: Invalid width in {ltype}_baseline.json. Skipping.")
                    continue
                self.baseline_widths[ltype] = float(width_m)
                self.map_baselines_to_3d_planes_from_data({ltype: baseline_data})
                planes_generated += 1
                width_summary.append(f"{ltype.replace('_', ' ').title()}: {width_m:.2f} m")
                self.message_text.append(f"Generated 3D plane for '{ltype}' → width {width_m:.2f} m")
            if planes_generated > 0:
                width_list = "\n".join(width_summary)
                self.message_text.append(f"Total 3D planes generated: {planes_generated}")
                self.message_text.append(f"Widths loaded:\n{width_list}")

        # ===============================================================
        # ROBUST POINT CLOUD LOADING — CHECKS ALL POSSIBLE SOURCES
        # ===============================================================
        pc_loaded = False

        # 1. Try worksheet config (full path)
        pc_file = config.get("point_cloud_file")
        if pc_file and os.path.exists(pc_file):
            try:
                self.load_point_cloud_from_path(pc_file)
                pc_loaded = True
                self.message_text.append(f"Point cloud loaded from worksheet config: {pc_file}")
            except Exception as e:
                self.message_text.append(f"Error loading point cloud from worksheet path: {str(e)}")

        # 2. If not → try layer config (may be relative or full)
        if not pc_loaded:
            layer_pc = layer_config.get("point_cloud_file")
            if layer_pc:
                # Try as full path first
                if os.path.exists(layer_pc):
                    candidate = layer_pc
                else:
                    # Try relative to worksheet root
                    candidate = os.path.join(worksheet_root, layer_pc)
                    candidate = os.path.normpath(candidate)

                if os.path.exists(candidate):
                    try:
                        self.load_point_cloud_from_path(candidate)
                        pc_loaded = True
                        self.message_text.append(f"Point cloud loaded from layer config: {candidate}")
                    except Exception as e:
                        self.message_text.append(f"Error loading point cloud from layer path: {str(e)}")

        # 3. Final fallback: ask user
        if not pc_loaded:
            self.message_text.append("No valid point cloud file found in config.")
            reply = QMessageBox.question(
                self,
                "Point Cloud Not Found",
                "The point cloud file for this worksheet could not be found.\n\n"
                "Would you like to select it manually now?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.Yes
            )
            if reply == QMessageBox.Yes:
                file_path, _ = QFileDialog.getOpenFileName(
                    self,
                    "Select Point Cloud File",
                    worksheet_root,
                    "Point Cloud Files (*.las *.laz *.ply *.bin)"
                )
                if file_path and os.path.exists(file_path):
                    try:
                        self.load_point_cloud_from_path(file_path)
                        pc_loaded = True
                        # Update worksheet config
                        config["point_cloud_file"] = file_path
                        self.current_worksheet_data["point_cloud_file"] = file_path
                        self.message_text.append(f"Point cloud manually loaded: {file_path}")
                    except Exception as e:
                        self.message_text.append(f"Failed to load selected point cloud: {str(e)}")
                else:
                    self.message_text.append("Point cloud loading cancelled.")
            else:
                self.message_text.append("Point cloud skipped.")

        # ===============================================================
        # UI setup & final messages
        # ===============================================================
        self.show_graph_section(category)

        is_construction_layer = (subfolder_type == "construction")
        if is_construction_layer:
            self.switch_to_construction_mode()
            self.zero_container.setVisible(False)
            self.scale_section.setVisible(True)
            self.message_text.append("Switched to Construction mode")
            self.construction_layer_info = {'folder': full_layer_path, 'name': layer_name}
            self.message_text.append(f"Construction layer ready: {layer_name}")
            self.message_text.append("You can now add Material Lines.")
        else:
            self.surface_container.setVisible(category == "Road")
            self.construction_container.setVisible(category == "Road")
            self.road_surface_container.setVisible(category == "Road")
            self.zero_container.setVisible(category in ["Road", "Bridge"])
            self.preview_button.setVisible(True)
            self.threed_map_button.setVisible(True)
            self.save_button.setVisible(True)

        self.add_layer_to_panel(layer_name, dimension)

        # Final summary
        self.message_text.append(f"Opened: {worksheet_name} → {subfolder_type}/{layer_name}")
        self.message_text.append(f"   • Baselines: {'All (solid)' if subfolder_type == 'designs' else f'{dotted_lines_drawn} reference(s) (dotted)'}")
        self.message_text.append(f"   • 3D Planes: {len(loaded_baselines)}")
        self.message_text.append(f"   • Zero Line: {'Loaded' if zero_loaded else 'Not loaded'}")
        self.message_text.append(f"   • Curve Labels: {len(self.curve_labels)} recreated")
        self.message_text.append(f"   • Point Cloud: {'Loaded' if pc_loaded else 'Not loaded'}")

        QMessageBox.information(self, "Worksheet Opened",
                                f"<b>{worksheet_name}</b> → {layer_name}\n\n"
                                f"Layer Type: {subfolder_type.capitalize()}\n"
                                f"Baselines: {'All loaded' if subfolder_type == 'designs' else f'{dotted_lines_drawn} reference(s)'}\n"
                                f"3D Planes: {len(loaded_baselines)}\n"
                                f"Curves: {len(self.curve_labels)} labels\n"
                                f"Point Cloud: {'Yes' if pc_loaded else 'No'}\n"
                                f"Zero Line: {'Yes' if zero_loaded else 'No'}")

        self.canvas.draw_idle()
        if hasattr(self, 'vtk_widget'):
            self.vtk_widget.GetRenderWindow().Render()

# ==========================================================================================================================================================
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
# ==========================================================================================================================================================

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
        if renderer:
            actors = renderer.GetActors()
            for actor in actors:
                if actor.GetUserData() and "source" in actor.GetUserData():
                    renderer.RemoveActor(actor)
            self.vtk_widget.GetRenderWindow().Render()

    
# ===========================================================================================================================================================

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


# ===========================================================================================================================================================

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

# ============================================================================================================================================================
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

# ===========================================================================================================================================================

    # New method to load baselines from JSON files
    def load_baselines_from_layer(self):
        if not self.current_layer_name:
            QMessageBox.warning(self, "No Layer", "No active design layer.")
            return {}

        layer_folder = os.path.join(
            self.WORKSHEETS_BASE_DIR,
            self.current_worksheet_name,
            "designs",
            self.current_layer_name
        )

        loaded_baselines = {}
        for ltype in self.baseline_types:
            json_filename = f"{ltype}_baseline.json"
            json_path = os.path.join(layer_folder, json_filename)
            if os.path.exists(json_path):
                try:
                    with open(json_path, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                    loaded_baselines[ltype] = data
                except Exception as e:
                    self.message_text.append(f"Error loading {ltype}: {str(e)}")

        return loaded_baselines
    
# ===========================================================================================================================================================

    def save_current_design_layer(self):
        """
        Save checked baselines + create realistic earthwork operation_config.json
        after road_surface is saved.
        """
        if not hasattr(self, 'current_worksheet_name') or not self.current_worksheet_name:
            QMessageBox.warning(self, "No Worksheet", "No active worksheet.")
            return
        if not hasattr(self, 'current_layer_name') or not self.current_layer_name:
            QMessageBox.warning(self, "No Layer", "No active design layer.")
            return

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
            QMessageBox.warning(self, "Zero Line Required", "Zero line must be set.")
            return

        # ── Load all baselines ───────────────────────────────────────────────
        surface_data = None
        construction_data = None
        road_surface_data = None

        for fname, key in [
            ("surface_baseline.json", "surface"),
            ("construction_baseline.json", "construction"),
            ("road_surface_baseline.json", "road_surface")
        ]:
            path = os.path.join(layer_folder, fname)
            if os.path.exists(path):
                try:
                    with open(path, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                    if key == "surface":
                        surface_data = data
                    elif key == "construction":
                        construction_data = data
                    elif key == "road_surface":
                        road_surface_data = data
                except Exception as e:
                    self.message_text.append(f"Warning: Could not load {fname}: {e}")

        saved_count = 0
        saved_files = []
        road_surface_just_saved = False

        dir_vec = self.zero_end_point - self.zero_start_point
        zero_length = self.total_distance
        ref_z = self.zero_start_z

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

            if ltype not in self.baseline_widths or self.baseline_widths[ltype] <= 0:
                QMessageBox.warning(
                    self,
                    "Width Missing",
                    f"No valid width for {ltype.replace('_', ' ').title()}.\n"
                    "Please set width via 'Map on 3D' first."
                )
                continue

            width_m = self.baseline_widths[ltype]

            baseline_data = {
                "baseline_type": ltype.replace('_', ' ').title(),
                "baseline_key": ltype,
                "color": self.line_types[ltype]['color'],
                "width_meters": float(width_m),
                "zero_line_start": self.zero_start_point.tolist(),
                "zero_line_end": self.zero_end_point.tolist(),
                "zero_start_elevation": float(ref_z),
                "total_chainage_length": float(zero_length),
                "polylines": []
            }

            for poly_2d in polylines:
                poly_3d_points = []
                for dist, rel_z in poly_2d:
                    t = dist / zero_length if zero_length > 0 else 0
                    pos_along = self.zero_start_point + t * dir_vec
                    abs_z = ref_z + rel_z

                    world_point = [float(pos_along[0]), float(pos_along[1]), float(abs_z)]

                    point_entry = {
                        "chainage_m": float(dist),
                        "chainage_str": self.format_chainage(dist, for_dialog=True),
                        "relative_elevation_m": float(rel_z),
                        "world_coordinates": world_point
                    }

                    # Height differences (reference only)
                    if ltype == "construction" and surface_data:
                        min_diff = float('inf')
                        surface_rel = None
                        for poly in surface_data.get("polylines", []):
                            for pt in poly.get("points", []):
                                d = abs(pt["chainage_m"] - dist)
                                if d < min_diff:
                                    min_diff = d
                                    surface_rel = pt["relative_elevation_m"]
                        if surface_rel is not None:
                            diff = abs(round(surface_rel - rel_z, 3))
                            point_entry["surface_to_construction_diff_m"] = diff

                    elif ltype == "road_surface" and construction_data:
                        min_diff = float('inf')
                        const_rel = None
                        for poly in construction_data.get("polylines", []):
                            for pt in poly.get("points", []):
                                d = abs(pt["chainage_m"] - dist)
                                if d < min_diff:
                                    min_diff = d
                                    const_rel = pt["relative_elevation_m"]
                        if const_rel is not None:
                            diff = abs(round(const_rel - rel_z, 3))
                            point_entry["construction_to_road_surface_diff_m"] = diff

                    poly_3d_points.append(point_entry)

                if len(poly_3d_points) >= 2:
                    start_ch = poly_3d_points[0]["chainage_m"]
                    end_ch = poly_3d_points[-1]["chainage_m"]
                    baseline_data["polylines"].append({
                        "start_chainage_m": float(start_ch),
                        "start_chainage_str": self.format_chainage(start_ch, for_dialog=True),
                        "end_chainage_m": float(end_ch),
                        "end_chainage_str": self.format_chainage(end_ch, for_dialog=True),
                        "points": poly_3d_points
                    })

            if not baseline_data["polylines"]:
                continue

            json_filename = f"{ltype}_baseline.json"
            json_path = os.path.join(layer_folder, json_filename)

            try:
                with open(json_path, 'w', encoding='utf-8') as f:
                    json.dump(baseline_data, f, indent=4, ensure_ascii=False)
                saved_count += 1
                saved_files.append(json_filename)

                if ltype == "road_surface":
                    road_surface_just_saved = True
                    road_surface_data = baseline_data  # update reference

            except Exception as e:
                QMessageBox.critical(self, "Save Failed", f"Error saving {json_filename}:\n{str(e)}")
                return

        # ────────────────────────────────────────────────────────────────
        #   After road surface save → add diff + realistic earthwork config
        # ────────────────────────────────────────────────────────────────
        if road_surface_just_saved and surface_data and road_surface_data:

            # 1. Add surface_to_road_surface_diff_m to road surface points
            for poly in road_surface_data["polylines"]:
                for pt in poly["points"]:
                    ch = pt["chainage_m"]
                    min_d = float('inf')
                    surf_rel = None
                    for s_poly in surface_data.get("polylines", []):
                        for s_pt in s_poly.get("points", []):
                            d = abs(s_pt["chainage_m"] - ch)
                            if d < min_d:
                                min_d = d
                                surf_rel = s_pt["relative_elevation_m"]
                    if surf_rel is not None:
                        abs_diff = abs(round(surf_rel - pt["relative_elevation_m"], 3))
                        pt["surface_to_road_surface_diff_m"] = abs_diff

            # Save updated road_surface
            rs_path = os.path.join(layer_folder, "road_surface_baseline.json")
            try:
                with open(rs_path, 'w', encoding='utf-8') as f:
                    json.dump(road_surface_data, f, indent=4, ensure_ascii=False)
                self.message_text.append("Updated road_surface_baseline.json with surface diff")
            except Exception as e:
                self.message_text.append(f"Warning: Could not update road surface json: {e}")

            # 2. Create realistic earthwork operation_config.json

            operations = []

            if road_surface_data.get("polylines"):
                main_poly = road_surface_data["polylines"][0]  # assuming single main alignment
                points = main_poly["points"]

                TOL_BALANCED = 0.20   # meters - adjust if needed
                TOL_SURF_ROAD = 0.04  # old small tolerance - can keep or merge with above

                # Get widths once
                w_construction = self.baseline_widths.get("construction", 20.0)
                w_road_surface = self.baseline_widths.get("road_surface", 12.0)

                for i in range(len(points) - 1):
                    p1 = points[i]
                    p2 = points[i + 1]

                    ch1 = p1["chainage_m"]
                    ch2 = p2["chainage_m"]
                    str1 = p1["chainage_str"]
                    str2 = p2["chainage_str"]

                    length = ch2 - ch1
                    if length <= 0:
                        continue

                    def get_abs_elev(baseline_data, chainage, ref_z):
                        min_d = float('inf')
                        rel_elev = None
                        for poly in baseline_data.get("polylines", []):
                            for pt in poly["points"]:
                                d = abs(pt["chainage_m"] - chainage)
                                if d < min_d:
                                    min_d = d
                                    rel_elev = pt["relative_elevation_m"]
                        if rel_elev is not None and min_d < 10.0:
                            return ref_z + rel_elev
                        return None

                    surf_z1   = get_abs_elev(surface_data,   ch1, ref_z)
                    surf_z2   = get_abs_elev(surface_data,   ch2, ref_z)
                    road_z1   = get_abs_elev(road_surface_data, ch1, ref_z)
                    road_z2   = get_abs_elev(road_surface_data, ch2, ref_z)
                    const_z1  = get_abs_elev(construction_data, ch1, ref_z) if construction_data else None
                    const_z2  = get_abs_elev(construction_data, ch2, ref_z) if construction_data else None

                    segment = {
                        "from_chainage_str": str1,
                        "to_chainage_str": str2,
                        "operation_type": "data_missing",
                        "cut_volume_ref_construction_m3": 0.0,
                        "cut_volume_ref_road_surface_m3": 0.0,
                        "digging_volume_m3": 0.0,
                        "width_construction_m": round(w_construction, 1),
                        "width_road_surface_m": round(w_road_surface, 1)
                    }

                    if surf_z1 is None or surf_z2 is None or road_z1 is None or road_z2 is None:
                        operations.append(segment)
                        continue

                    avg_surf = (surf_z1 + surf_z2) / 2
                    avg_road = (road_z1 + road_z2) / 2
                    height_surf_road = avg_surf - avg_road

                    # ── New priority: check construction closeness first ────────
                    is_balanced_by_construction = False

                    if const_z1 is not None and const_z2 is not None:
                        avg_const = (const_z1 + const_z2) / 2
                        height_surf_const = avg_surf - avg_const
                        if abs(height_surf_const) <= TOL_BALANCED:
                            is_balanced_by_construction = True

                    cut_vol_constr = 0.0
                    cut_vol_road   = 0.0
                    digging_vol    = 0.0

                    if is_balanced_by_construction:
                        operation = "balanced"
                    elif height_surf_road > TOL_SURF_ROAD:
                        operation = "cutting"

                        # Cut to construction (if available)
                        if const_z1 is not None and const_z2 is not None:
                            avg_const = (const_z1 + const_z2) / 2
                            h_constr = avg_surf - avg_const
                            if h_constr > 0:
                                cut_vol_constr = round(h_constr * w_construction * length, 2)

                        # Cut to road surface
                        h_road = height_surf_road
                        cut_vol_road = round(h_road * w_road_surface * length, 2)

                    elif height_surf_road < -TOL_SURF_ROAD:
                        operation = "digging"
                        h_dig = -height_surf_road
                        digging_vol = round(h_dig * w_road_surface * length, 2)
                    else:
                        operation = "balanced"

                    segment["operation_type"] = operation

                    if operation == "cutting":
                        segment["cut_volume_ref_construction_m3"] = cut_vol_constr
                        segment["cut_volume_ref_road_surface_m3"] = cut_vol_road
                    elif operation == "digging":
                        segment["digging_volume_m3"] = digging_vol
                    # balanced → all volumes stay 0.0

                    operations.append(segment)

            # Sort segments by chainage string
            operations.sort(key=lambda x: x["from_chainage_str"])

            config_data = {
                "zero_line_start": self.zero_start_point.tolist(),
                "zero_line_end": self.zero_end_point.tolist(),
                "zero_start_elevation": float(ref_z),
                "total_chainage_length": float(zero_length),
                "volume_method": "Average End Area (Trapezoidal)",
                "segments": operations
            }

            config_path = os.path.join(layer_folder, "operation_config.json")
            try:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(config_data, f, indent=4, ensure_ascii=False)
                saved_files.append("operation_config.json")
                self.message_text.append("Created earthwork operation_config.json")
            except Exception as e:
                self.message_text.append(f"Warning: Could not create operation_config.json: {e}")
                
        # ── Final feedback ───────────────────────────────────────────────────
        if saved_count > 0:
            file_list = "\n".join([f"• {f}" for f in saved_files])
            self.message_text.append(f"Saved/updated {saved_count} baseline(s) + config:")
            self.message_text.append(file_list)

            QMessageBox.information(
                self,
                "Save Successful",
                f"Saved/updated {saved_count} baseline(s) and created earthwork config.\n\n"
                f"Files:\n{file_list}\n\nLocation:\n{layer_folder}"
            )
        else:
            QMessageBox.information(self, "Nothing Saved", "No valid baselines to save.")


# ===========================================================================================================================================================
    def map_baselines_to_3d_planes_from_data(self, loaded_baselines):
        """Generate 3D planes from loaded baseline data — NOW WITH REAL CURVES using self.curve_labels"""
        if not self.zero_line_set:
            self.message_text.append("Zero line not set - cannot generate 3D planes.")
            return

        import numpy as np
        import vtk

        #self.clear_baseline_planes()

        planes_generated = 0
        width_summary = []

        # Build curve lookup for fast access: chainage -> angle + direction
        curve_lookup = {}
        for item in self.curve_labels:
            ch = item['chainage']
            cfg = item['config']
            curve_lookup[ch] = {
                'angle': cfg['angle'],
                'left_turn': cfg['inner_curve']  # inner = left turn
            }

        for ltype, baseline_data in loaded_baselines.items():
            width_m = baseline_data.get("width_meters")
            if width_m is None or width_m <= 0:
                self.message_text.append(f"Invalid width for {ltype}. Skipping.")
                continue

            self.baseline_widths[ltype] = float(width_m)
            half_width = width_m / 2.0
            width_summary.append(f"{ltype.replace('_', ' ').title()}: {width_m:.2f} m")

            rgba = self.plane_colors.get(ltype, (0.0, 0.8, 0.0, 0.4))
            color_rgb = rgba[:3]
            opacity = rgba[3]

            # Start from zero line start
            current_pos = np.array(self.zero_start_point)
            current_dir = (self.zero_end_point - self.zero_start_point)
            dir_len = np.linalg.norm(current_dir)
            if dir_len > 0:
                current_dir /= dir_len

            left_points = []
            right_points = []

            for poly in baseline_data.get("polylines", []):
                points = poly.get("points", [])
                if len(points) < 2:
                    continue

                # Dense sampling for smooth curve
                for i in range(len(points) - 1):
                    pt1 = points[i]
                    pt2 = points[i + 1]

                    ch1 = pt1["chainage_m"]
                    ch2 = pt2["chainage_m"]
                    
                    # Use stored relative elevation (height above zero line) directly
                    # Fallback to world coord calc only if missing (for legacy support)
                    z1 = pt1.get("relative_elevation_m", pt1["world_coordinates"][2] - self.zero_start_z)
                    z2 = pt2.get("relative_elevation_m", pt2["world_coordinates"][2] - self.zero_start_z)

                    # Sample segment
                    samples = max(10, int((ch2 - ch1) / 0.5) + 1)

                    # Calculate scaling factor for angle (distribute rotation over the segment)
                    step_angle = 0
                    step_rotation_matrix = None
                    
                    if ch1 in curve_lookup:
                        angle = curve_lookup[ch1]['angle']
                        left_turn = curve_lookup[ch1]['left_turn']
                        
                        # Convert total angle to radians
                        total_rad = np.deg2rad(angle)
                        if not left_turn:
                            total_rad = -total_rad
                            
                        # Calculate angle per sample step
                        if samples > 1:
                            step_rad = total_rad / (samples - 1)
                            step_angle = step_rad
                            
                            # Pre-calculate rotation matrix for one step
                            sin_val = np.sin(step_rad)
                            cos_val = np.cos(step_rad)
                            step_rotation_matrix = np.array([
                                [cos_val, -sin_val, 0],
                                [sin_val, cos_val, 0],
                                [0, 0, 1]
                            ])

                    # Sample segment
                    for s in range(samples):
                        t = s / (samples - 1) if samples > 1 else 0
                        ch = ch1 + t * (ch2 - ch1)
                        rel_z = z1 + t * (z2 - z1)
                        # Use current 3D position Z (which tracks slope) + relative elevation
                        abs_z = current_pos[2] + rel_z

                        # Advance position AND direction
                        if s > 0:
                            # 1. Rotate direction if we are in a curve
                            if step_rotation_matrix is not None:
                                current_dir = step_rotation_matrix @ current_dir
                                # Normalize to prevent drift
                                norm = np.linalg.norm(current_dir)
                                if norm > 0:
                                    current_dir /= norm

                            # 2. Move forward
                            step = (ch2 - ch1) / (samples - 1)
                            current_pos += current_dir * step

                        center = np.array([current_pos[0], current_pos[1], abs_z])

                        # Perpendicular
                        perp = np.array([-current_dir[1], current_dir[0], 0.0])
                        if np.linalg.norm(perp) > 0:
                            perp /= np.linalg.norm(perp)

                        left = center + perp * half_width
                        right = center - perp * half_width

                        left_points.append(left)
                        right_points.append(right)

            # Create smooth continuous surface
            if len(left_points) >= 2:
                points = vtk.vtkPoints()
                cells = vtk.vtkCellArray()
                for i in range(len(left_points)):
                    points.InsertNextPoint(left_points[i])
                    points.InsertNextPoint(right_points[i])
                for i in range(len(left_points) - 1):
                    quad = vtk.vtkQuad()
                    quad.GetPointIds().SetId(0, 2*i)
                    quad.GetPointIds().SetId(1, 2*i + 1)
                    quad.GetPointIds().SetId(2, 2*(i+1) + 1)
                    quad.GetPointIds().SetId(3, 2*(i+1))
                    cells.InsertNextCell(quad)

                polydata = vtk.vtkPolyData()
                polydata.SetPoints(points)
                polydata.SetPolys(cells)

                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputData(polydata)

                actor = vtk.vtkActor()
                actor.SetMapper(mapper)
                actor.GetProperty().SetColor(*color_rgb)
                actor.GetProperty().SetOpacity(opacity)

                self.renderer.AddActor(actor)
                self.baseline_plane_actors.append(actor)
                planes_generated += 1

        # Render
        rw = (self.vtk_widget if hasattr(self, 'vtk_widget') else self.vtkWidget).GetRenderWindow()
        rw.Render()

        if planes_generated > 0:
            width_list = "\n".join(width_summary)
            self.message_text.append(f"Generated {planes_generated} smooth curved 3D plane(s)")
            self.message_text.append(f"Widths: {width_list}")
        else:
            self.message_text.append("No 3D planes generated.")

# ==========================================================================================================================================================
    def save_baseline_with_curves(self, ltype, json_path):
        """
        Saves a single baseline to JSON, including curve angles only if curve labels exist.
        Called from save_current_design_layer() for each checked baseline.
        """
        if not self.line_types[ltype]['polylines']:
            return False

        dir_vec = self.zero_end_point - self.zero_start_point
        zero_length = self.total_distance
        ref_z = self.zero_start_z

        def format_chainage_str(chainage_m: float) -> str:
            if not hasattr(self, 'zero_start_km') or self.zero_start_km is None:
                return f"{chainage_m:.3f}m"
            km_part = int(self.zero_start_km)
            metres_part = chainage_m % 1000
            return f"{km_part}+{metres_part:03.0f}"

        baseline_data = {
            "baseline_type": ltype.replace('_', ' ').title(),
            "baseline_key": ltype,
            "color": self.line_types[ltype]['color'],
            "width_meters": float(self.baseline_widths.get(ltype, 0.0)),
            "zero_line_start": self.zero_start_point.tolist(),
            "zero_line_end": self.zero_end_point.tolist(),
            "zero_start_elevation": float(ref_z),
            "total_chainage_length": float(zero_length),
            "polylines": []
        }

        # Add polylines with world coordinates
        for poly_2d in self.line_types[ltype]['polylines']:
            poly_3d_points = []
            for dist, rel_z in poly_2d:
                t = dist / zero_length if zero_length > 0 else 0
                pos_along = self.zero_start_point + t * dir_vec
                abs_z = ref_z + rel_z
                world_point = [float(pos_along[0]), float(pos_along[1]), float(abs_z)]
                poly_3d_points.append({
                    "chainage_m": float(dist),
                    "chainage_str": format_chainage_str(dist),
                    "relative_elevation_m": float(rel_z),
                    "world_coordinates": world_point
                })

            if len(poly_3d_points) >= 2:
                start_ch = poly_3d_points[0]["chainage_m"]
                end_ch = poly_3d_points[-1]["chainage_m"]
                baseline_data["polylines"].append({
                    "start_chainage_m": float(start_ch),
                    "start_chainage_str": format_chainage_str(start_ch),
                    "end_chainage_m": float(end_ch),
                    "end_chainage_str": format_chainage_str(end_ch),
                    "points": poly_3d_points
                })

        # === ONLY ADD CURVES IF THEY EXIST ===
        if self.curve_labels:
            baseline_data["curves"] = []
            for item in self.curve_labels:
                curve_entry = {
                    "chainage_m": float(item['chainage']),
                    "angle_deg": float(item['config']['angle']),
                    "inner_curve": bool(item['config']['inner_curve']),
                    "outer_curve": bool(item['config']['outer_curve'])
                }
                baseline_data["curves"].append(curve_entry)

        # Save to file
        try:
            with open(json_path, 'w', encoding='utf-8') as f:
                json.dump(baseline_data, f, indent=4)
            return True
        except Exception as e:
            self.message_text.append(f"Failed to save {os.path.basename(json_path)}: {str(e)}")
            return False
        
# ===========================================================================================================================================================
    def load_baseline_with_curves(self, json_path, ltype):
        """
        Loads a single baseline JSON file.
        - If 'curves' field exists → recreates yellow curve labels on 2D graph.
        - If no 'curves' → scans points for embedded 'angle_deg' and recreates labels from those.
        Always restores saved width and polylines.
        """
        if not os.path.exists(json_path):
            return False

        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            self.message_text.append(f"Error reading {os.path.basename(json_path)}: {str(e)}")
            return False

        # Restore width
        width = data.get("width_meters")
        if width is not None and width > 0:
            self.baseline_widths[ltype] = float(width)

        # Restore polylines (for 2D redraw and 3D mapping)
        polylines_2d = []
        for poly in data.get("polylines", []):
            poly_2d = []
            for pt in poly.get("points", []):
                dist = pt.get("chainage_m", 0.0)
                rel_z = pt.get("relative_elevation_m", 0.0)
                poly_2d.append((dist, rel_z))
            if len(poly_2d) >= 2:
                polylines_2d.append(poly_2d)
        self.line_types[ltype]['polylines'] = polylines_2d

        recreated = False

        # First, check for separate "curves" list (older format)
        if "curves" in data:
            for curve in data["curves"]:
                config = {
                    'angle': curve.get("angle_deg", 5.0),
                    'inner_curve': curve.get("inner_curve", False),
                    'outer_curve': curve.get("outer_curve", False)
                }
                chainage = curve.get("chainage_m", 0.0)
                self.add_curve_label_at_x(chainage, config)
            self.message_text.append(f"Recreated {len(data['curves'])} curve label(s) from separate 'curves' in {os.path.basename(json_path)}")
            recreated = True

        # If no "curves", scan points for embedded angles (your JSON format)
        else:
            recreated_count = 0
            for poly in data.get("polylines", []):
                for pt in poly.get("points", []):
                    if "angle_deg" in pt:
                        chainage = pt.get("chainage_m", 0.0)
                        config = {
                            'angle': pt["angle_deg"],
                            'inner_curve': pt.get("inner_curve", False),
                            'outer_curve': pt.get("outer_curve", False)
                        }
                        # Add to curve_labels list (global)
                        self.curve_labels.append({'chainage': chainage, 'config': config})
                        # Recreate yellow label on 2D graph
                        self.add_curve_label_at_x(chainage, config)
                        recreated_count += 1

            if recreated_count > 0:
                self.message_text.append(f"Recreated {recreated_count} curve label(s) from embedded point angles in {os.path.basename(json_path)}")
                recreated = True

        if not recreated:
            self.message_text.append(f"Loaded straight baseline: {os.path.basename(json_path)} (no curves found)")

        return True
# ===========================================================================================================================================================
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
                # Check if bridge baseline is active
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
                    self.drawing_zero_line = True
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
        """Update both the top Chainage Scale and the main graph X-axis with KM+interval ticks.
        Now includes sub-interval (half-interval) minor ticks for better readability."""
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

        interval = self.zero_interval
        half_interval = interval / 2.0

        # Generate positions: major at 0, interval, 2*interval, ... and minor at half_interval, 1.5*interval, ...
        max_pos = self.total_distance
        major_positions = np.arange(0, max_pos + interval / 2, interval)  # + half to include last if close
        minor_positions = np.arange(half_interval, max_pos + interval / 2, interval)

        # Filter to stay within bounds
        major_positions = major_positions[major_positions <= max_pos]
        minor_positions = minor_positions[minor_positions <= max_pos]

        # All tick positions for grid/minor lines (both major and minor)
        all_tick_positions = np.sort(np.unique(np.concatenate([major_positions, minor_positions])))

        # Generate labels only for major ticks
        tick_labels = []
        current_km = self.zero_start_km
        for pos in major_positions:
            meters_into_line = pos
            km_offset = int(meters_into_line // 1000)
            chainage_m = int(meters_into_line % 1000)
            label = f"{current_km + km_offset}+{chainage_m:03d}"
            tick_labels.append(label)

        # === UPDATE TOP CHAINAGE SCALE (Orange bar) ===
        if hasattr(self, 'scale_ax') and self.scale_ax:
            self.scale_ax.clear()
            self.scale_ax.plot([0, self.total_distance], [0, 0], color='black', linewidth=3)
            self.scale_ax.set_xlim(0, self.total_distance)
            self.scale_ax.set_ylim(-0.1, 1.2)
            self.scale_ax.set_yticks([])

            # Major ticks with labels
            self.scale_ax.set_xticks(major_positions)
            self.scale_ax.set_xticklabels(tick_labels, rotation=15, ha='right', fontsize=8)

            # Minor ticks (no labels)
            self.scale_ax.set_xticks(minor_positions, minor=True)

            # Style minor ticks
            self.scale_ax.tick_params(axis='x', which='major', length=10, width=1.5, colors='black')
            self.scale_ax.tick_params(axis='x', which='minor', length=6, width=1, colors='gray')

            self.scale_ax.set_title("Chainage Scale", fontsize=10, pad=10, color='#D35400')
            self.scale_ax.spines['bottom'].set_visible(True)
            self.scale_ax.spines['top'].set_visible(False)
            self.scale_ax.spines['left'].set_visible(False)
            self.scale_ax.spines['right'].set_visible(False)
            self.scale_ax.set_facecolor('#FFE5B4')  # Light orange background
            if hasattr(self, 'scale_canvas'):
                self.scale_canvas.draw_idle()

        # === UPDATE MAIN GRAPH X-AXIS ===
        self.ax.set_xlim(0, self.total_distance)

        # Major ticks with labels
        self.ax.set_xticks(major_positions)
        self.ax.set_xticklabels(tick_labels, rotation=15, ha='right', fontsize=8)

        # Minor ticks (sub-intervals)
        self.ax.set_xticks(minor_positions, minor=True)

        # Style ticks
        self.ax.tick_params(axis='x', which='major', length=8, width=1.2)
        self.ax.tick_params(axis='x', which='minor', length=5, width=0.8, color='gray')

        self.ax.set_xlabel('Chainage (KM + Interval)', fontsize=10, labelpad=10)
        self.ax.grid(True, axis='x', which='major', linestyle='--', alpha=0.7, linewidth=1.2)
        self.ax.grid(True, axis='x', which='minor', linestyle=':', alpha=0.4, linewidth=0.8)

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
    def on_key_press(self, event):
        if event.key == 'escape' and self.active_line_type and self.current_points:
            self.finish_current_polyline()
            self.message_text.append(f"{self.active_line_type.replace('_', ' ').title()} completed with Escape key")

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

# =========================================================================================================================================================
# # CURVE BUTTON HANDLER
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


# =========================================================================================================================================================
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

# ==============================================================================================================================================================
    def add_curve_label_at_x(self, x, config=None):
        """
        Add a curve label at position x.
        config is a dict: {'angle': 5.0, 'outer_curve': True, 'inner_curve': False}
        If None, uses self.current_curve_config (no default value).
        """
        if config is None:
            if hasattr(self, 'current_curve_config') and self.current_curve_config:
                config = self.current_curve_config
            else:
                self.message_text.append("No curve configuration set. Cannot add label.")
                return

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

        # Store artist, config, and chainage
        self.curve_labels.append({'artist': label, 'config': config, 'chainage': x})

        # Reconnect picker
        if self.curve_pick_id:
            self.canvas.mpl_disconnect(self.curve_pick_id)

        def on_pick(ev):
            # Find which label was clicked
            for item in self.curve_labels:
                if ev.artist == item['artist']:
                    self.edit_individual_curve_label(item['artist'], item['config'], item['chainage'])
                    break

        self.curve_pick_id = self.canvas.mpl_connect('pick_event', on_pick)

# ===========================================================================================================================================================
    # New: Build curved alignment based on curve labels
    def build_curved_alignment(self, sample_interval=1.0):
        """
        Builds a curved polyline in 3D based on zero line and curve deflections from labels.
        Returns: list of (chainage, pos_3d, dir_3d) tuples.
        """
        import numpy as np

        if not hasattr(self, 'zero_line_set') or not self.zero_line_set:
            self.message_text.append("Zero line not set.")
            return []

        # Sort curve data by chainage
        sorted_curves = sorted(self.curve_labels, key=lambda item: item['chainage'])

        # Initial position and direction from zero line
        zero_start = np.array(self.start_point) if hasattr(self, 'start_point') else np.array([0,0,0])  # Fallback
        zero_end = np.array(self.end_point) if hasattr(self, 'end_point') else np.array([1,0,0])
        init_dir = zero_end - zero_start
        init_dir_len = np.linalg.norm(init_dir)
        if init_dir_len > 0:
            init_dir /= init_dir_len
        else:
            init_dir = np.array([1.0, 0.0, 0.0])  # Default X direction

        total_ch = self.total_distance

        # Sample chainages
        ch_samples = np.arange(0, total_ch + sample_interval, sample_interval)

        alignment = []  # List of (ch, pos_3d, dir_3d)

        current_pos = zero_start.copy()
        current_dir = init_dir.copy()
        current_ch = 0.0
        curve_idx = 0  # Track next curve

        for next_ch in ch_samples[1:]:
            segment_length = next_ch - current_ch

            # Check for curve at or near current_ch
            while curve_idx < len(sorted_curves) and sorted_curves[curve_idx]['chainage'] <= current_ch + 0.01:  # Small tolerance
                curve = sorted_curves[curve_idx]['config']
                angle_deg = curve['angle']
                angle_rad = np.deg2rad(angle_deg)

                # Determine direction: inner='left', outer='right', both=average or choose one (simplify to left if inner, right if outer, left if both)
                if curve['inner_curve'] and curve['outer_curve']:
                    direction = 'left'  # Arbitrary choice, or handle as bank only (no horizontal turn)
                elif curve['inner_curve']:
                    direction = 'left'
                elif curve['outer_curve']:
                    direction = 'right'
                else:
                    direction = 'left'  # Default

                # Rotate current_dir horizontally (XY plane)
                cos_a = np.cos(angle_rad)
                sin_a = np.sin(angle_rad) if direction == 'left' else -np.sin(angle_rad)
                rot_matrix = np.array([
                    [cos_a, -sin_a, 0],
                    [sin_a, cos_a, 0],
                    [0, 0, 1]
                ])
                current_dir = rot_matrix @ current_dir

                # Normalize
                dir_len = np.linalg.norm(current_dir)
                if dir_len > 0:
                    current_dir /= dir_len

                curve_idx += 1

            # Advance position
            delta_pos = current_dir * segment_length
            current_pos += delta_pos

            # Interpolate base Z from zero line (linear along chainage)
            t = current_ch / total_ch if total_ch > 0 else 0
            base_z = zero_start[2] + t * (zero_end[2] - zero_start[2])
            current_pos[2] = base_z

            alignment.append((current_ch, current_pos.copy(), current_dir.copy()))

            current_ch = next_ch

        # Add end if needed
        if current_ch < total_ch:
            remaining = total_ch - current_ch
            delta_pos = current_dir * remaining
            current_pos += delta_pos
            t = total_ch / total_ch
            base_z = zero_start[2] + t * (zero_end[2] - zero_start[2])
            current_pos[2] = base_z
            alignment.append((total_ch, current_pos.copy(), current_dir.copy()))

        return alignment
    
# ===========================================================================================================================================================
    def edit_individual_curve_label(self, label_artist, current_config, chainage):
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
            QMessageBox.warning(self, "Invalid Angle", "Angle must be > 0.")
            return

        # Format new display text
        outer = new_config['outer_curve']
        inner = new_config['inner_curve']
        curve_type = "O&I" if outer and inner else ("O" if outer else ("I" if inner else ""))
        new_text = f"{new_angle:.1f}° - {curve_type}" if curve_type else f"{new_angle:.1f}°"

        # Update only this label's text
        label_artist.set_text(new_text)

        # Update stored config for this label
        for item in self.curve_labels:
            if item['artist'] == label_artist:
                item['config'] = new_config
                break

        self.message_text.append(f"Updated curve label to: {new_text} at chainage {chainage:.2f}")
        self.canvas.draw_idle()

# ===========================================================================================================================================================
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

# ===========================================================================================================================================================
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


# ===========================================================================================================================================================
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

# ===========================================================================================================================================================
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

# ===========================================================================================================================================================
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

# ===========================================================================================================================================================
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

# ===========================================================================================================================================================
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

# ===========================================================================================================================================================
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
    
# ===========================================================================================================================================================
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

# ===========================================================================================================================================================
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
            # Final update before hiding
            self.update_progress(100, "Loading complete!")
            QTimer.singleShot(100, self.hide_progress_bar)
        except Exception as e:
            self.hide_progress_bar()


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

# ========================================================================================================
# Define function for the mesurement type:
    def set_measurement_type(self, m_type):
        """Set the measurement type and configure the UI and state accordingly."""
        if not self.measurement_active:
            return
        self.measurement_active = True
        self.current_measurement = m_type
        self.measurement_started = False
        self.plotting_active = True
        # Only clear measurement points if not continuing measurement line
        if m_type != 'vertical_line' or not hasattr(self, 'measurement_line_points'):
            self.measurement_points = []
        # Configure based on measurement type
        if m_type == 'polygon':
            self.complete_polygon_button.setVisible(True)
            self.complete_polygon_button.setStyleSheet("""
            QPushButton {
                background-color: #98FB98; /* Light green */
                color: black;
                border: 1px solid gray;
                padding: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #7DCE7D; /* Slightly darker green on hover */
            }
            """)
            self.measurement_started = True
        elif m_type == 'horizontal_line':
            self.presized_button.setVisible(True)
        elif m_type == 'vertical_line':
                print("Vertical line selected. Click 2 points (A, B) for height measurement.")
                self.measurement_points = [] # Clear previous points
                self.presized_button.setVisible(True)
                return
        else:
            # Hide the presized button for other measurement types
            self.presized_button.setVisible(False)

# =================================================================================================================================
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
                            if '=' in text: # Angle label
                                continue # Don't modify angle labels
              
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

# ==================================================================================================================================
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
    
# ==================================================================================================================================
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
    
# ==================================================================================================================================
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
    
# ==================================================================================================================================
# Define function to add sphere marker:
    def add_sphere_marker(self, point, label=None, radius = 0.07, color="Red"):
        """Add a sphere marker at the specified position with optional label"""
        sphere = vtkSphereSource()
        sphere.SetRadius(radius)
        sphere.SetCenter(point[0], point[1], point[2])
        mapper = vtkPolyDataMapper()
        mapper.SetInputConnection(sphere.GetOutputPort())
        actor = vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(self.colors.GetColor3d(color)) # Use specified color
        if hasattr(self, 'measurement_points'):
            actor.point_index = len(self.measurement_points) - 1 # Store reference
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
            text_actor.GetProperty().SetColor(self.colors.GetColor3d("White"))
            self.renderer.AddActor(text_actor)
            text_actor.SetCamera(self.renderer.GetActiveCamera())
            self.measurement_actors.append(text_actor)
        self.vtk_widget.GetRenderWindow().Render()
        return actor # Return the sphere actor
    
# ==================================================================================================================================
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
                display_coords.append(display_coord[:2]) # Only need x,y
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
    
# =========================================================================================================================================
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
            if direction_norm > 1e-10: # Small threshold to avoid division by zero
                direction = direction / direction_norm
                perpendicular = np.cross(direction, [0, 0, 1]) # Cross with Z-axis for perpendicular vector
                perpendicular_norm = np.linalg.norm(perpendicular)
  
                if perpendicular_norm > 1e-10: # Check if perpendicular vector is valid
                    perpendicular = perpendicular / perpendicular_norm
                    label_pos = midpoint + perpendicular * 0.5 # Small offset
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
            text_actor.SetScale(0.5, 0.5, 0.5) # Increased size to 0.5
            text_actor.AddPosition(label_pos[0], label_pos[1], label_pos[2])
            text_actor.GetProperty().SetColor(self.colors.GetColor3d("Blue")) # Blue color
            self.renderer.AddActor(text_actor)
            text_actor.SetCamera(self.renderer.GetActiveCamera())
            self.measurement_actors.append(text_actor)
            self.vtk_widget.GetRenderWindow().Render()
            return actor
        
# ==================================================================================================================================
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
        
# =========================================================================================================
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
            
# ==================================================================================================================================
    def on_click(self, obj, event):
        if self.drawing_zero_line:
            interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
            pos = interactor.GetEventPosition()
            # Pick point accurately
            cell_picker = vtk.vtkCellPicker()
            cell_picker.SetTolerance(0.0005)
            cell_picker.Pick(pos[0], pos[1], 0, self.renderer)
            clicked_point = None
            if cell_picker.GetCellId() != -1:
                clicked_point = np.array(cell_picker.GetPickPosition())
                points = np.asarray(self.point_cloud.points)
                if len(points) > 0:
                    distances = np.sum((points - clicked_point)**2, axis=1)
                    nearest_idx = np.argmin(distances)
                    clicked_point = points[nearest_idx]
            if clicked_point is None:
                clicked_point = self.find_nearest_point_in_neighborhood(pos)
            if clicked_point is None:
                return
            self.zero_points.append(clicked_point)
            label = "Start" if len(self.zero_points) == 1 else "End"
            actor = self.add_sphere_marker(clicked_point, label, color="purple")
            self.temp_zero_actors.append(actor)
            # When both points are picked
            if len(self.zero_points) == 2:
                self.drawing_zero_line = False
                # Open dialog with current picked points
                dialog = ZeroLineDialog(
                    point1=self.zero_points[0],
                    point2=self.zero_points[1],
                    km1=None,
                    chain1=None,
                    km2=None,
                    chain2=None,
                    interval=20.0,
                    parent=self
                )
                if dialog.exec_() == QDialog.Accepted:
                    try:
                        # Get updated coordinates from dialog (user can edit them)
                        p1, p2 = dialog.get_points()
                        if p1 is None or p2 is None:
                            raise ValueError("Invalid or empty coordinates for one or both points.")

                        # Update zero line points and state
                        self.zero_start_point = np.array(p1, dtype=float)
                        self.zero_end_point = np.array(p2, dtype=float)
                        self.zero_start_z = float(p1[2])  # Reference elevation from Point 1 Z

                        # Read KM values (optional, can be empty → None)
                        km1_text = dialog.km1_edit.text().strip()
                        km2_text = dialog.km2_edit.text().strip()
                        self.zero_start_km = int(km1_text) if km1_text else None
                        self.zero_end_km = int(km2_text) if km2_text else None

                        # Read interval (default 20.0 if empty)
                        # interval_text = dialog.interval_edit.text().strip()
                        # self.zero_interval = float(interval_text) if interval_text else 20.0
                        # #self.zero_interval = int(dialog.interval_edit.text().strip() or "20")


                        # === Read and validate Interval (must be positive integer) ===
                        interval_text = dialog.interval_edit.text().strip()
                        try:
                            interval_val = int(interval_text)
                            if interval_val <= 0:
                                raise ValueError("Interval must be greater than 0")
                            self.zero_interval = interval_val
                        except (ValueError, TypeError):
                            self.zero_interval = 20  # Safe default
                            self.message_text.append("Warning: Invalid interval entered. Using default 20m.")
                        
                        # Calculate total distance
                        self.total_distance = np.linalg.norm(self.zero_end_point - self.zero_start_point)
                        self.original_total_distance = self.total_distance
                        self.zero_line_set = True

                        # Update 3D actors (keep the temporary spheres as final markers)
                        self.zero_start_actor = self.temp_zero_actors[0]
                        self.zero_end_actor = self.temp_zero_actors[1]
                        self.zero_line_actor = self.add_line_between_points(
                            self.zero_start_point, self.zero_end_point, "purple", show_label=False
                        )

                        # Update 2D graph
                        self.ax.set_xlim(0, self.total_distance)
                        if hasattr(self, 'zero_graph_line') and self.zero_graph_line:
                            self.zero_graph_line.remove()
                        self.zero_graph_line, = self.ax.plot([0, self.total_distance], [0, 0],
                                                            color='purple', linewidth=3)

                        # Update ticks and scale
                        self.update_chainage_ticks()
                        self.update_scale_ticks()
                        if hasattr(self, 'scale_section'):
                            self.scale_section.setVisible(True)

                        # Redraw
                        self.canvas.draw()
                        self.figure.tight_layout()

                        # === SAVE ZERO LINE CONFIG TO JSON IN CURRENT DESIGN LAYER ===
                        self.save_zero_line_config_to_current_layer()

                        # Success messages
                        self.message_text.append("✓ Zero Line configured and saved successfully!")
                        self.message_text.append(f"   Length: {self.total_distance:.2f} m")
                        self.message_text.append(f"   Interval: {self.zero_interval:.1f} m")
                        if self.zero_start_km is not None:
                            self.message_text.append(f"   Start KM: {self.zero_start_km}+000")

                    except ValueError as e:
                        QMessageBox.warning(self, "Invalid Input", f"{str(e)}")
                        self.reset_zero_drawing()
                else:
                    # Dialog cancelled
                    self.reset_zero_drawing()
                    self.message_text.append("Zero line configuration cancelled.")

            self.vtk_widget.GetRenderWindow().Render()
            return  # Important: return early if we were drawing zero line
#-------------------------------------
        if not self.measurement_active or not self.current_measurement or self.freeze_view or not self.plotting_active:
            return
        interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        pos = interactor.GetEventPosition()
        # First try using cell picker which can pick between points
        cell_picker = vtk.vtkCellPicker()
        cell_picker.SetTolerance(0.0005) # Small tolerance for accurate picking
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
            return # No point found
        # Handle vertical line case (after measurement line is set)
        if self.current_measurement == 'vertical_line':
            # Add point to measurement_points (for vertical line)
            self.measurement_points.append(clicked_point)
            point_label = chr(65 + len(self.measurement_points) - 1) # A, B
            # Visualize the point
            self.add_sphere_marker(clicked_point, point_label, color="Blue")
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
        if self.current_measurement == 'polygon' :
            # For first point, just add it
            if len(self.measurement_points) == 0:
                self.measurement_points.append(clicked_point)
                point_label = 'A' # First point is always A
                self.add_sphere_marker(clicked_point, point_label)
  
                self.vtk_widget.GetRenderWindow().Render()
                return
      
            # For regular clicks (adding points)
            point_label = chr(65 + len(self.measurement_points)) # A, B, C, etc.
            self.measurement_points.append(clicked_point)
            self.add_sphere_marker(clicked_point, point_label)
            # If this is at least the second point, draw a line
            if len(self.measurement_points) >= 2:
                p1 = self.measurement_points[-2]
                p2 = self.measurement_points[-1]
                self.add_line_between_points(p1, p2, "Purple")
               
        interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        self.vtk_widget.GetRenderWindow().Render()
        
# ===========================================================================================================================================================
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
        
# =================================================================================================================
# Define function for the connect the points which is already drawn on point cloud data:
    def connect_signals(self):
        # Connect UI signals
        self.reset_action_button.clicked.connect(self.reset_action)
        self.reset_all_button.clicked.connect(self.reset_all)
        self.preview_button.clicked.connect(self.on_curve_button_clicked)
        self.threed_map_button.clicked.connect(self.map_baselines_to_3d_planes)
        self.save_button.clicked.connect(self.save_current_design_layer)
        
        # Connect button signals that were commented out
        self.create_project_button.clicked.connect(self.open_create_project_dialog)
        self.existing_worksheet_button.clicked.connect(self.open_existing_worksheet)

        self.new_worksheet_button.clicked.connect(self.open_new_worksheet_dialog)
        self.new_design_button.clicked.connect(self.open_create_new_design_layer_dialog)
        self.new_construction_button.clicked.connect(self.open_construction_new_dialog)
        self.new_measurement_button.clicked.connect(self.open_measurement_dialog)
        self.load_button.clicked.connect(self.load_point_cloud)
        self.help_button.clicked.connect(self.show_help_dialog)
        
        # Connect logout button and update user name label
        self.logout_button.clicked.connect(self.logout_user)
        self.user_name_label.setText(f"👤 {self.current_user_full_name}")
        
        # Connect checkbox signals
        self.zero_line.stateChanged.connect(lambda state: self.on_checkbox_changed(state, 'zero'))
        self.surface_baseline.stateChanged.connect(lambda state: self.on_checkbox_changed(state, 'surface'))
        self.construction_line.stateChanged.connect(lambda state: self.on_checkbox_changed(state, 'construction'))
        self.road_surface_line.stateChanged.connect(lambda state: self.on_checkbox_changed(state, 'road_surface'))
        self.bridge_zero_line.stateChanged.connect(lambda state: self.on_checkbox_changed(state, 'zero'))
        self.projection_line.stateChanged.connect(lambda state: self.on_checkbox_changed(state, 'projection_line'))
        self.construction_dots_line.stateChanged.connect(lambda state: self.on_checkbox_changed(state, 'construction_dots'))
        self.deck_line.stateChanged.connect(lambda state: self.on_checkbox_changed(state, 'deck_line'))
        
        # Connect pencil button signals
        self.zero_pencil.clicked.connect(self.edit_zero_line)
        self.bridge_zero_pencil.clicked.connect(self.edit_zero_line)
        self.construction_dots_pencil.clicked.connect(self.edit_construction_dots_line)
        self.deck_pencil.clicked.connect(self.edit_deck_line)
        self.add_material_line_button.clicked.connect(self.open_material_line_dialog)
   
        self.volume_slider.valueChanged.connect(self.volume_changed)

        # Connect the Start/Stop button in the zoom toolbar to material drawing
        self.start_stop_button.toggled.connect(self.on_material_drawing_toggle)

        # # Connect graph click for drawing new points (construction, material, etc.)
        if hasattr(self, 'canvas') and self.canvas:
            self.graph_click_id = self.canvas.mpl_connect('button_press_event', self.on_graph_click)
            # print("Graph click event connected")  # Debug - remove later

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

# ===========================================================================================================================================================
    def logout_user(self):
        """Handle user logout - shows confirmation and returns to login page"""
        from PyQt5.QtWidgets import QMessageBox
        
        # Show confirmation dialog
        reply = QMessageBox.question(
            self,
            "Confirm Logout",
            f"Are you sure you want to logout, {self.current_user_full_name}?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            # Close current window
            self.close()
            
            # Import and show login dialog
            from login import LoginDialog
            from welcome_page import WelcomePage
            from PyQt5.QtWidgets import QDialog
            
            login_dlg = LoginDialog()
            if login_dlg.exec_() == LoginDialog.Accepted:
                # Get new user data
                username = login_dlg.logged_in_username
                user_id = login_dlg.logged_in_user_id
                
                # Get user full name from user config file
                user_full_name = username
                try:
                    user_config_path = os.path.join(r"E:\3D_Tool\user", user_id, "user_config.txt")
                    if os.path.exists(user_config_path):
                        with open(user_config_path, 'r', encoding='utf-8') as f:
                            user_config = json.load(f)
                            user_full_name = user_config.get("full_name", username)
                except Exception:
                    pass
                
                # Show Welcome Page after successful login
                welcome_page = WelcomePage(user_full_name=user_full_name)
                if welcome_page.exec_() != QDialog.Accepted:
                    # User closed welcome page without clicking Start Now
                    QApplication.quit()
                    return
                
                # Create new PointCloudViewer window with new user - in fullscreen
                new_window = PointCloudViewer(username=username, user_id=user_id, user_full_name=user_full_name)
                new_window.showFullScreen()
            else:
                # User cancelled login - exit application
                QApplication.quit()

# ===========================================================================================================================================================
    def on_slider_changed(self, value):
        """Handle slider change with graph scrolling"""
        # Call the original volume_changed logic
        print(f"Volume slider changed to: {value}%")
        if self.zero_line_set:
            self.update_scale_marker()
        
        # Scroll the graph
        self.scroll_graph_with_slider(value)
        
        # Also update the main graph marker if zero line is set
        if self.zero_line_set:
            self.update_main_graph_marker(value)

# ==================================================================================================================================
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
        self.message_text.append(f"--- Vertical Height ---\n AB = {height:.2f} {units_suffix}")
        # Check if point B is on baseline
        on_baseline = False
        if hasattr(self, 'baseline_actors') and self.baseline_actors:
            baseline_actor = self.baseline_actors[0]
            bounds = baseline_actor.GetBounds()
            baseline_z = bounds[4] # Minimum Z of the baseline plane
            if abs(point_b[2] - baseline_z) < 0.001: # Account for floating point precision
                on_baseline = True
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
        self.vtk_widget.GetRenderWindow().Render()
        
# ==================================================================================================================================
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
        self.message_text.append(f"--- Horizontal line ---\n PQ: {distance:.2f} {units_suffix}")
        
# ===========================================================================================================================
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
            label = chr(65 + (i+1)%n) # B, C, etc.
            angles.append((label, angle))
        # Calculate area using triangulation (Delaunay triangulation)
        try:
            from scipy.spatial import Delaunay
            # Project points to 2D plane (using the two most significant dimensions)
            abs_normal = np.abs(normal)
            if abs_normal[0] > abs_normal[1] and abs_normal[0] > abs_normal[2]:
                # Most aligned with YZ plane
                coords = projected_points[:, [1, 2]] # Use Y and Z coordinates
            elif abs_normal[1] > abs_normal[2]:
                # Most aligned with XZ plane
                coords = projected_points[:, [0, 2]] # Use X and Z coordinates
            else:
                # Most aligned with XY plane (default)
                coords = projected_points[:, [0, 1]] # Use X and Y coordinates
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
            self.message_text.append("Note: scipy not available, using shoelace formula instead")
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
        # Add angle labels to the 3D view
        for i in range(n):
            a = points[i]
            b = points[(i+1)%n]
            c = points[(i+2)%n]
            angle_label = angles[i][1]
            self.add_angle_label(b, a, c, f"{angle_label:.1f}°", offset=0.8)
        # Output results
        self.polygon_area_meters = area_meters
        self.polygon_perimeter_meters = perimeter_meters
        self.message_text.append(f"Polygon Surface Area = {area:.2f} {area_suffix}")
        self.message_text.append(f"Polygon Perimeter = {perimeter:.2f} {perimeter_suffix}")
        
# ==================================================================================================================================
    def complete_polygon(self):
        self.plotting_active = False
        if self.current_measurement != 'polygon' and self.current_measurement != 'round_pillar_polygon':
            return
        if len(self.measurement_points) < 3:
            self.message_text.append("Need at least 3 points to complete a polygon")
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
        # Process the polygon measurement
        if self.current_measurement == 'polygon' :
            self.process_polygon_measurement()
        # Hide the Complete Polygon button after completing
        self.complete_polygon_button.setVisible(False)
        self.complete_polygon_button.setStyleSheet("") # Reset to default style
        self.vtk_widget.GetRenderWindow().Render()
        
# ==================================================================================================================================
#Define the function for the handle the Presized button action:
    def handle_presized_button(self):
        """Handle the Presized button click for round pillar measurement."""
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
                    surface_area = surface_area_meters * 10000 # m² to cm²
                    volume = volume_meters * 1000000 # m³ to cm³
                    outer_surface = outer_surface_meters * 10000 # m² to cm²
                    perimeter = perimeter_meters * 100
                    area_suffix = "square cm"
                    volume_suffix = "cubic cm"
                    perimeter_suffix = "centi meter"
                elif units_suffix == "mm":
                    surface_area = surface_area_meters * 1000000 # m² to mm²
                    volume = volume_meters * 1000000000 # m³ to mm³
                    outer_surface = outer_surface_meters * 1000000 # m² to mm²
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
                self.message_text.append(f"Surface Area of Polygon = {surface_area:.2f} {area_suffix}\n")
                self.message_text.append(f"Volume of Polygon = {volume:.2f} {volume_suffix}\n")
                self.message_text.append(f"Outer Surface Area of Polygon= {outer_surface:.2f} {area_suffix}")
            except Exception as e:
                self.message_text.append(f"Error creating presized polygon volume measurements")
            self.vtk_widget.GetRenderWindow().Render()
            return
        if (hasattr(self, 'current_measurement') and self.current_measurement == 'vertical_line' and \
            hasattr(self, 'measurement_points') and len(self.measurement_points) >= 2):
            # Vertical line case
            try:
                self.create_presized_vertical_line()
                # self.output_list.addItem("Presized vertical line created")
  
            except Exception as e:
                self.message_text.append(f"Error creating presized vertical line")
        if (hasattr(self, 'current_measurement') and self.current_measurement == 'horizontal_line' and \
            hasattr(self, 'measurement_points') and len(self.measurement_points) >= 2):
            # Horizontal line case
            try:
                self.create_presized_horizontal_line()
                # self.output_list.addItem("Presized horizontal line created")
  
            except Exception as e:
                self.message_text.append(f"Error creating presized horizontal line")
                
# ===========================================================================================================================
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
            self.presized_horizontal_points = [point_p.tolist(), point_r.tolist()] # [P, R]
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
                # self.output_list.addItem("Horizontal line color changed to LightGrey")
            # 2. Change point Q sphere color to LightGrey
            if hasattr(self, 'point_q_actor') and self.point_q_actor is not None:
                self.point_q_actor.GetProperty().SetColor(self.colors.GetColor3d("LightGrey"))
                # self.output_list.addItem("Point Q sphere color changed to LightGrey")
            # 3. Change point Q label color to LightGrey
            for actor in self.measurement_actors:
                if isinstance(actor, vtk.vtkFollower):
                    try:
                        text_source = actor.GetMapper().GetInputConnection(0, 0).GetProducer()
                        if isinstance(text_source, vtk.vtkVectorText) and text_source.GetText() == "Q":
                            actor.GetProperty().SetColor(self.colors.GetColor3d("LightGrey"))
                            # self.output_list.addItem("Point Q label color changed to LightGrey")
                            break
                    except:
                        continue
            # 4. Change distance label color to LightGrey and reposition it above the original PQ line
            if hasattr(self, 'horizontal_distance_label_actor') and self.horizontal_distance_label_actor is not None:
                self.horizontal_distance_label_actor.GetProperty().SetColor(self.colors.GetColor3d("LightGrey"))
                # self.output_list.addItem("Horizontal distance label color changed to LightGrey and repositioned")
            # Draw the new straight horizontal line from P to R in red
            self.presized_horizontal_line_actor = self.add_line_between_points(point_p, point_r, "Red", f"PR={distance:.2f}{units_suffix}")
            # Add point R marker with label
            self.point_r_actor = self.add_sphere_marker(point_r, "R", color="Red")
            return distance_meters
        except Exception as e:
            self.message_text.append(f"Error creating presized horizontal line: {str(e)}")
            
# ===========================================================================================================================
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
            self.presized_vertical_points = [point_a.tolist(), point_c.tolist()] # [A, C]
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
                # self.output_list.addItem("Vertical line AB color changed to LightGrey")
            # 2. Change point B sphere color to LightGrey
            if hasattr(self, 'point_b_actor') and self.point_b_actor is not None:
                self.point_b_actor.GetProperty().SetColor(self.colors.GetColor3d("LightGrey"))
                # self.output_list.addItem("Point B sphere color changed to LightGrey")
            # 3. Change point B label color to LightGrey
            for actor in self.measurement_actors:
                if isinstance(actor, vtk.vtkFollower):
                    try:
                        text_source = actor.GetMapper().GetInputConnection(0, 0).GetProducer()
                        if isinstance(text_source, vtk.vtkVectorText) and text_source.GetText() == "B":
                            actor.GetProperty().SetColor(self.colors.GetColor3d("LightGrey"))
                            # self.output_list.addItem("Point B label color changed to LightGrey")
                            break
                    except:
                        continue
            # 4. Change distance label color to LightGrey
            if hasattr(self, 'distance_label_actor') and self.distance_label_actor is not None:
                self.distance_label_actor.GetProperty().SetColor(self.colors.GetColor3d("LightGrey"))
                # self.output_list.addItem("Vertical distance label color changed to LightGrey and repositioned")
            # Draw the new vertical line from A to C in red
            self.presized_line_actor = self.add_line_between_points(point_a, point_c, "Red",f"AC={distance:.2f}{units_suffix}")
            # Add point C marker with label
            self.point_c_actor = self.add_sphere_marker(point_c, "C", color="Red")
            self.message_text.append(f"Presized Vertical Height AC: {distance:.2f} {units_suffix}")
            self.vtk_widget.GetRenderWindow().Render()
            return distance_meters
        except Exception as e:
            self.message_text.append(f"Error creating presized vertical line: {str(e)}")
            
# ==========================================================================================================================
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

# ============================================================================================================================
# RESET ACTION (Modified)
    def reset_action(self):
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

            self.clear_baseline_planes()
            self.clear_reference_lines()          # Clear previous 2D lines
            self.clear_reference_actors()         # Clear previous 3D actors
            
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
            
# ============================================================================================================================
# Define the function for the reset all:

    def reset_all(self):
        """Reset ALL lines from the graph"""
        # First call reset_action to handle any current drawing
        self.reset_action()

        # Clear graph
        self.ax.cla()
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(-5, 5)
        self.canvas.draw_idle()

        # Clear 3D view
        if hasattr(self, 'renderer') and self.renderer:
            actors = self.renderer.GetActors()
            actors.InitTraversal()
            actor = actors.GetNextItem()
            while actor:
                self.renderer.RemoveActor(actor)
                actor = actors.GetNextItem()
            self.renderer.ResetCamera()
        if hasattr(self, 'vtk_widget') and self.vtk_widget:
            self.vtk_widget.GetRenderWindow().Render()
        

# Clear baseline planes and other 3D actors
        self.clear_baseline_planes()
        self.baseline_plane_actors = []
        self.material_3d_actors = {}
        self.curve_3d_actors = []

        # Hide and clear frames/sections
        if hasattr(self, 'three_d_frame'):
            self.three_d_frame.setVisible(False)
            self.three_d_frame.clear() if hasattr(self.three_d_frame, 'clear') else None
        if hasattr(self, 'two_d_frame'):
            self.two_d_frame.setVisible(False)
            self.two_d_frame.clear() if hasattr(self.two_d_frame, 'clear') else None
        if hasattr(self, 'merger_frame'):
            self.merger_frame.setVisible(False)
            self.merger_frame.clear() if hasattr(self.merger_frame, 'clear') else None

        # Clear bottom section (assuming it has widgets/artists)
        if hasattr(self, 'bottom_section'):
            self.bottom_section.setVisible(False)
            for widget in self.bottom_section.findChildren(QWidget):
                widget.deleteLater()

        # Clear scale section (chainage ticks, zero line, etc.)
        if hasattr(self, 'scale_section'):
            self.scale_section.setVisible(False)
            for widget in self.scale_section.findChildren(QWidget):
                widget.deleteLater()
        self.zero_line_set = False
        if hasattr(self, 'zero_graph_line') and self.zero_graph_line:
            self.zero_graph_line.remove()
            self.zero_graph_line = None

        # Clear left section panels (checkboxes, containers)
        for container in [self.surface_container, self.construction_container, self.road_surface_container,
                          self.zero_container, self.deck_line_container, self.projection_container,
                          self.construction_dots_container, getattr(self, 'bridge_zero_container', None)]:
            if container:
                container.setVisible(False)
                for widget in container.findChildren(QWidget):
                    widget.deleteLater()

        # Clear material widgets and artists
        for widget in self.material_line_widgets:
            widget.deleteLater()
        self.material_line_widgets = []
        self.material_polylines = {}
        self.material_polylines_artists = {}
        self.material_fill_patches = {}
        self.material_segment_labels = {}
        self.material_configs = []

        # Clear other states
        self.line_types = {k: {'polylines': [], 'artists': []} for k in self.line_types}
        self.baseline_widths = {}
        self.current_layer_name = None
        self.current_worksheet_name = None
        self.current_project_name = None
        self.current_worksheet_data = {}
        self.construction_mode_active = False
        self.material_drawing_active = False
        self.active_material_index = None
        self.active_line_type = None
        self.curve_active = False
        self.measurement_points = []

        # Clear message text
        self.message_text.clear()

        # Hide banner/mode if exists
        if hasattr(self, 'mode_banner'):
            self.mode_banner.setVisible(False)

        # Reset display worksheet
        if hasattr(self, 'worksheet_display'):
            self.worksheet_display.clear()

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


# --------------------------------------------------------------------        
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
        self.threed_map_button.setVisible(False)
        self.save_button.setVisible(False)
        # ===== END OF ADDED SECTION =====
        
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
                self.scale_section.setVisible(True)
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
        
        # Call the original measurement reset logic (keeping your existing code)
        # Reset view and plotting states
        self.freeze_view = False
        self.plotting_active = True
        
        # Reset cropped data
        if hasattr(self, 'cropped_cloud'):
            self.cropped_cloud = None
        
        self.current_measurement = None
        
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
        # Reset UI state
        self.measurement_active = True

        # Hide scale section
        if hasattr(self, 'scale_section'):
            self.scale_section.setVisible(False)

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

        # Reset button
        self.preview_button.setText("Curve")
        self.preview_button.setStyleSheet("""
            QPushButton { background-color: #808080; color: white; padding: 12px; border-radius: 6px;
                          font-size: 15px; font-weight: bold; }
            QPushButton:hover { background-color: #6E6E6E; }
        """)

        # Clear 3D curve actors
        for actor in self.curve_3d_actors:
            self.renderer.RemoveActor(actor)
        self.curve_3d_actors.clear()
        self.curve_start_point_3d = None

# ===========================================================================================================================================================
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

# ===========================================================================================================================================
    def preview_lines_on_3d(self):
        if not self.zero_line_set:
            self.message_text.append("Zero line must be set before previewing.")
            return
        has_polylines = any(self.line_types[lt]['polylines'] for lt in ['surface', 'construction', 'road_surface'])
        if not has_polylines:
            self.message_text.append("No lines drawn to preview.")
            return

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

# ===========================================================================================================================================================
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



# =================================================================================================================================
# MAP ROAD BASELINES TO 3D PLANES

    def map_baselines_to_3d_planes(self):
        """
        Universal mapping method:
        - If any point has angle_deg → curved road (cumulative horizontal turns applied to X/Y path)
        - If no angles → straight road (original behaviour)
        Forces user to enter width for each checked baseline.
        Uses original world_coordinates as base but adjusts for cumulative turns.
        """
        import numpy as np
        import vtk

        if not self.zero_line_set:
            QMessageBox.warning(self, "Zero Line Required", "Please set the Zero Line first.")
            return
        
        ref_z = self.zero_start_z if hasattr(self, 'zero_start_z') else self.zero_start_point[2]

        baseline_checkboxes = {
            'surface': self.surface_baseline,
            'construction': self.construction_line,
            'road_surface': self.road_surface_line,
            'deck_line': self.deck_line,
            'projection_line': self.projection_line,
        }

        checked_types = [ltype for ltype, cb in baseline_checkboxes.items() if cb.isChecked()]

        if not checked_types:
            QMessageBox.information(self, "Nothing Selected", "No baselines are checked for mapping.")
            return

        # Force user to enter width for each checked baseline
        for ltype in checked_types:
            current_width = self.baseline_widths.get(ltype)

            dialog = RoadPlaneWidthDialog(current_width=current_width, parent=self)
            display_name = ltype.replace('_', ' ').title()
            dialog.setWindowTitle(f"Enter Width for {display_name} Baseline")

            if dialog.exec_() != QDialog.Accepted:
                self.message_text.append(f"✗ Mapping cancelled for {display_name}.")
                continue

            width = dialog.get_width()
            if width is None or width <= 0:
                QMessageBox.warning(self, "Invalid Width", f"Please enter a valid width > 0 for {display_name}.")
                continue

            self.baseline_widths[ltype] = width
            self.message_text.append(f"✓ Width confirmed → {display_name}: {width:.2f} m")

        valid_types = [ltype for ltype in checked_types if self.baseline_widths.get(ltype) is not None]
        if not valid_types:
            QMessageBox.information(self, "No Valid Widths", "No baselines have a valid width. Mapping aborted.")
            return

        # Collect 2D polylines
        current_polylines = {}
        total_new_segments = 0
        for ltype in valid_types:
            polylines = self.line_types[ltype]['polylines']
            if polylines:
                current_polylines[ltype] = polylines
                total_new_segments += sum(len(p) - 1 for p in polylines if len(p) >= 2)

        if total_new_segments == 0:
            QMessageBox.information(self, "No Data", "No line segments found in selected baselines.")
            return

        # AUTO DETECT: Curved if any angle in polylines (from loaded or current)
        has_curves = any(
            any("angle_deg" in pt for poly in current_polylines.get(ltype, []) for pt in poly)
            for ltype in valid_types
        )
        mode_text = "curved road" if has_curves else "straight road"
        self.message_text.append(f"Mapping as {mode_text} (curves detected: {has_curves}).")

        plane_count_this_time = 0
        width_summary = []

        for ltype, polylines in current_polylines.items():
            width = self.baseline_widths[ltype]
            half_width = width / 2.0
            width_summary.append(f"{ltype.replace('_', ' ').title()}: {width:.2f} m")

            rgba = self.plane_colors.get(ltype, (0.5, 0.5, 0.5, 0.4))
            color_rgb = rgba[:3]
            opacity = rgba[3]

            for poly_2d in polylines:
                if len(poly_2d) < 2:
                    continue

                # For curved: build cumulative direction from angles at points
                current_pos = np.array(self.zero_start_point)
                current_dir = (self.zero_end_point - self.zero_start_point) / self.total_distance if self.total_distance > 0 else np.array([1.0, 0.0, 0.0])
                all_left_pts = []
                all_right_pts = []

                for i in range(len(poly_2d) - 1):
                    dist1, rel_z1 = poly_2d[i]
                    dist2, rel_z2 = poly_2d[i + 1]

                    # Check for angle at start of segment (pt1)
                    angle_deg1 = next((curve['config']['angle'] for curve in self.curve_labels if abs(curve['chainage'] - dist1) < 0.5), 0.0)
                    inner1 = next((curve['config']['inner_curve'] for curve in self.curve_labels if abs(curve['chainage'] - dist1) < 0.5), False)
                    outer1 = next((curve['config']['outer_curve'] for curve in self.curve_labels if abs(curve['chainage'] - dist1) < 0.5), False)

                    # Apply horizontal turn if angle at this point
                    if angle_deg1 > 0:
                        angle_rad = np.deg2rad(angle_deg1)
                        direction = 'left' if inner1 else ('right' if outer1 else 'left')
                        sin_a = np.sin(angle_rad) if direction == 'left' else -np.sin(angle_rad)
                        cos_a = np.cos(angle_rad)
                        rot_matrix = np.array([
                            [cos_a, -sin_a, 0],
                            [sin_a, cos_a, 0],
                            [0, 0, 1]
                        ])
                        current_dir = rot_matrix @ current_dir
                        current_dir /= np.linalg.norm(current_dir) if np.linalg.norm(current_dir) > 0 else 1

                    # Sample segment finely for smooth curve (1m steps)
                    sample_step = 1.0
                    ch_samples = np.arange(dist1, dist2 + sample_step, sample_step)
                    if ch_samples[-1] != dist2:
                        ch_samples = np.append(ch_samples, dist2)

                    for j in range(len(ch_samples)):
                        ch = ch_samples[j]
                        t_seg = (ch - dist1) / (dist2 - dist1) if (dist2 - dist1) > 0 else 0
                        rel_z = rel_z1 + t_seg * (rel_z2 - rel_z1)
                        abs_z = ref_z + rel_z

                        # Advance position along current_dir
                        seg_length = min(sample_step, dist2 - ch) if j < len(ch_samples) - 1 else 0
                        delta_pos = current_dir * seg_length
                        current_pos += delta_pos

                        center = np.array([current_pos[0], current_pos[1], abs_z])

                        # Perp for width
                        horiz = np.array([current_dir[0], current_dir[1], 0.0])
                        hlen = np.linalg.norm(horiz)
                        if hlen < 1e-6:
                            perp = np.array([0, 1, 0])
                        else:
                            horiz /= hlen
                            perp = np.array([-horiz[1], horiz[0], 0.0])

                        left = center + perp * half_width
                        right = center - perp * half_width

                        all_left_pts.append(left)
                        all_right_pts.append(right)

                # Create continuous actor from all points
                if len(all_left_pts) >= 2:
                    actor = self.create_vtk_quad_strip(all_left_pts, all_right_pts, color=color_rgb, opacity=opacity)
                    if actor:
                        self.renderer.AddActor(actor)
                        self.baseline_plane_actors.append(actor)
                        plane_count_this_time += 1

        # Render
        rw = (self.vtk_widget if hasattr(self, 'vtk_widget') else self.vtkWidget).GetRenderWindow()
        rw.Render()

        # Summary
        width_list = "\n".join(width_summary)
        curve_info = f" with {sum(1 for poly in polylines for _, _ in poly if 'angle_deg' in locals())} curve point(s)" if has_curves else ""

        self.message_text.append(f"Successfully mapped {plane_count_this_time} plane segments to 3D{curve_info}.")
        self.message_text.append(f"Widths applied:\n{width_list}")

        QMessageBox.information(
            self,
            "Mapping Complete",
            f"Added {plane_count_this_time} plane segments{curve_info}.\n\n"
            f"Widths used:\n{width_list}\n\n"
            "Click 'Save' to store these baselines permanently with the entered widths."
        )

    # =================================================================
        # New: create_vtk_polyline
    def create_vtk_polyline(self, points_3d, color=(1.0, 0.0, 0.0)):
        import vtk
        vtk_points = vtk.vtkPoints()
        for p in points_3d:
            vtk_points.InsertNextPoint(p)

        polyline = vtk.vtkPolyLine()
        polyline.GetPointIds().SetNumberOfIds(len(points_3d))
        for i in range(len(points_3d)):
            polyline.GetPointIds().SetId(i, i)

        cells = vtk.vtkCellArray()
        cells.InsertNextCell(polyline)

        polydata = vtk.vtkPolyData()
        polydata.SetPoints(vtk_points)
        polydata.SetLines(cells)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(polydata)

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(color)
        actor.GetProperty().SetLineWidth(3)
        return actor

# ==========================================================================================================================================================
    # New: create_vtk_quad_strip (for plane-like surface)
    def create_vtk_quad_strip(self, pts1, pts2, color=(0.0, 1.0, 0.0), opacity=1.0):
        import vtk
        if len(pts1) != len(pts2) or len(pts1) < 2:
            return None

        points = vtk.vtkPoints()
        for i in range(len(pts1)):
            points.InsertNextPoint(pts1[i])
            points.InsertNextPoint(pts2[i])

        quads = vtk.vtkCellArray()
        for i in range(len(pts1) - 1):
            quad = vtk.vtkQuad()
            quad.GetPointIds().SetId(0, 2 * i)
            quad.GetPointIds().SetId(1, 2 * (i + 1))
            quad.GetPointIds().SetId(2, 2 * (i + 1) + 1)
            quad.GetPointIds().SetId(3, 2 * i + 1)
            quads.InsertNextCell(quad)

        polydata = vtk.vtkPolyData()
        polydata.SetPoints(points)
        polydata.SetPolys(quads)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(polydata)

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(color)
        actor.GetProperty().SetOpacity(opacity)
        return actor
    # ===========================================================================================================================================================
    def clear_baseline_planes(self):
        """Clear all baseline plane actors from the 3D view"""
        if hasattr(self, 'renderer') and self.renderer:
            for actor in self.baseline_plane_actors:
                if self.renderer.HasViewProp(actor):
                    self.renderer.RemoveActor(actor)
        self.baseline_plane_actors = []
        if hasattr(self, 'vtk_widget') and self.vtk_widget:
            self.vtk_widget.GetRenderWindow().Render()

    # ===========================================================================================================================================================
    def clear_current_design_layer(self):
        """Clear all baselines from graph and 3D view to start a new design layer.
        This preserves the point cloud data and worksheet state."""
        
        self.message_text.append("Clearing current design layer...")
        
        # 1. Clear 2D graph baseline artists
        for line_type in self.line_types:
            for artist in self.line_types[line_type]['artists']:
                try:
                    artist.remove()
                except Exception:
                    pass
            self.line_types[line_type]['artists'] = []
            self.line_types[line_type]['polylines'] = []
        
        # 2. Clear 3D baseline plane actors
        self.clear_baseline_planes()
        
        # 3. Clear material-related artists from 2D graph
        for idx in list(self.material_polylines_artists.keys()):
            for artist in self.material_polylines_artists.get(idx, []):
                try:
                    artist.remove()
                except Exception:
                    pass
        self.material_polylines_artists = {}
        self.material_polylines = {}
        
        # 4. Clear material fill patches
        if hasattr(self, 'material_fill_patches'):
            for idx in list(self.material_fill_patches.keys()):
                for patch_dict in self.material_fill_patches.get(idx, []):
                    try:
                        if 'bg' in patch_dict:
                            patch_dict['bg'].remove()
                        if 'hatch' in patch_dict:
                            patch_dict['hatch'].remove()
                    except Exception:
                        pass
            self.material_fill_patches = {}
        
        # 5. Clear 3D material actors
        if hasattr(self, 'material_3d_actors') and self.renderer:
            for idx in list(self.material_3d_actors.keys()):
                for actor in self.material_3d_actors.get(idx, []):
                    try:
                        if self.renderer.HasViewProp(actor):
                            self.renderer.RemoveActor(actor)
                    except Exception:
                        pass
            self.material_3d_actors = {}
        
        # 6. Clear curve-related items from 2D graph
        if hasattr(self, 'curve_labels'):
            for label in self.curve_labels:
                try:
                    label.remove()
                except Exception:
                    pass
            self.curve_labels = []
        
        if self.curve_annotation:
            try:
                self.curve_annotation.remove()
            except Exception:
                pass
            self.curve_annotation = None
        
        if self.curve_arrow_annotation:
            try:
                self.curve_arrow_annotation.remove()
            except Exception:
                pass
            self.curve_arrow_annotation = None
        
        # 7. Clear 3D curve actors
        if hasattr(self, 'curve_3d_actors') and self.renderer:
            for actor in self.curve_3d_actors:
                try:
                    if self.renderer.HasViewProp(actor):
                        self.renderer.RemoveActor(actor)
                except Exception:
                    pass
            self.curve_3d_actors = []
        
        # 8. Clear material segment labels
        if hasattr(self, 'material_segment_labels'):
            for idx in list(self.material_segment_labels.keys()):
                for label in self.material_segment_labels.get(idx, []):
                    try:
                        label.remove()
                    except Exception:
                        pass
            self.material_segment_labels = {}
        
        # 9. Clear ALL text annotations from the graph (length labels, distance tags, etc.)
        if hasattr(self, 'ax') and self.ax:
            # Remove all text annotations
            for text in list(self.ax.texts):
                try:
                    text.remove()
                except Exception:
                    pass
            
            # Clear the legend if present
            legend = self.ax.get_legend()
            if legend:
                try:
                    legend.remove()
                except Exception:
                    pass
        
        # 10. Clear road plane actors from 3D view (the planes along zero line)
        if hasattr(self, 'renderer') and self.renderer:
            # Clear road plane actors (left + right planes along zero line)
            if hasattr(self, 'road_plane_actors'):
                for actor in self.road_plane_actors:
                    try:
                        if self.renderer.HasViewProp(actor):
                            self.renderer.RemoveActor(actor)
                    except Exception:
                        pass
                self.road_plane_actors = []
            
            # Clear road plane center actor
            if hasattr(self, 'road_plane_center_actor') and self.road_plane_center_actor:
                try:
                    if self.renderer.HasViewProp(self.road_plane_center_actor):
                        self.renderer.RemoveActor(self.road_plane_center_actor)
                except Exception:
                    pass
                self.road_plane_center_actor = None
            
            # Clear slider marker actor
            if hasattr(self, 'slider_marker_actor') and self.slider_marker_actor:
                try:
                    if self.renderer.HasViewProp(self.slider_marker_actor):
                        self.renderer.RemoveActor(self.slider_marker_actor)
                except Exception:
                    pass
                self.slider_marker_actor = None
        
        # 11. Clear Zero Line from 2D graph
        if hasattr(self, 'zero_graph_line') and self.zero_graph_line:
            try:
                self.zero_graph_line.remove()
            except Exception:
                pass
            self.zero_graph_line = None
        
        # 10. Clear Zero Line actors from 3D view
        if hasattr(self, 'renderer') and self.renderer:
            # Remove zero line actor
            if hasattr(self, 'zero_line_actor') and self.zero_line_actor:
                try:
                    if self.renderer.HasViewProp(self.zero_line_actor):
                        self.renderer.RemoveActor(self.zero_line_actor)
                except Exception:
                    pass
                self.zero_line_actor = None
            
            # Remove zero start point actor
            if hasattr(self, 'zero_start_actor') and self.zero_start_actor:
                try:
                    if self.renderer.HasViewProp(self.zero_start_actor):
                        self.renderer.RemoveActor(self.zero_start_actor)
                except Exception:
                    pass
                self.zero_start_actor = None
            
            # Remove zero end point actor
            if hasattr(self, 'zero_end_actor') and self.zero_end_actor:
                try:
                    if self.renderer.HasViewProp(self.zero_end_actor):
                        self.renderer.RemoveActor(self.zero_end_actor)
                except Exception:
                    pass
                self.zero_end_actor = None
            
            # Remove temporary zero actors
            for actor in getattr(self, 'temp_zero_actors', []):
                try:
                    if self.renderer.HasViewProp(actor):
                        self.renderer.RemoveActor(actor)
                except Exception:
                    pass
            self.temp_zero_actors = []
        
        # 11. Reset Zero Line state variables
        self.zero_line_set = False
        self.zero_start_point = None
        self.zero_end_point = None
        self.zero_start_km = None
        self.zero_start_chain = None
        self.zero_end_km = None
        self.zero_end_chain = None
        self.zero_interval = None
        self.zero_physical_dist = 0.0
        self.zero_start_z = 0.0
        self.drawing_zero_line = False
        self.zero_points = []
        
        # 12. Reset state variables (but keep worksheet and point cloud)
        self.baseline_widths = {}
        self.curve_active = False
        self.current_curve_text = ""
        self.curve_start_x = None
        self.material_drawing_active = False
        self.active_material_index = None
        self.active_line_type = None
        
        # 13. Reset checkboxes to unchecked state
        try:
            self.zero_line.setChecked(False)
            self.surface_baseline.setChecked(False)
            self.construction_line.setChecked(False)
            self.road_surface_line.setChecked(False)
            if hasattr(self, 'bridge_zero_line'):
                self.bridge_zero_line.setChecked(False)
            if hasattr(self, 'deck_line'):
                self.deck_line.setChecked(False)
            if hasattr(self, 'projection_line'):
                self.projection_line.setChecked(False)
            if hasattr(self, 'construction_dots_line'):
                self.construction_dots_line.setChecked(False)
        except Exception:
            pass
        
        # 11. Redraw 2D canvas
        self.canvas.draw_idle()
        
        # 12. Re-render 3D view
        if hasattr(self, 'vtk_widget') and self.vtk_widget:
            self.vtk_widget.GetRenderWindow().Render()
        
        self.message_text.append("Design layer cleared. Ready to draw new baselines.")

# ============================================================================================================================================================

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

# =============================================================================================================================================================
    def resizeEvent(self, event):
        super(PointCloudViewer, self).resizeEvent(event)
        if self.vtk_widget:
            self.vtk_widget.GetRenderWindow().Render()

# =============================================================================================================================================================
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
        

# ===========================================================================================================================================================
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

# ===========================================================================================================================================================
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

# ===========================================================================================================================================================
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

# ===========================================================================================================================================================
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

# =============================================================================================================================================================
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

# ========================================================================================================================================================
    def get_current_design_layer_path(self):
        """Return path to current active design layer, fallback to worksheet root"""
        if not hasattr(self, 'current_worksheet_name') or not self.current_worksheet_name:
            self.message_text.append("No active worksheet — cannot determine layer path.")
            return None

        worksheet_path = os.path.join(self.WORKSHEETS_BASE_DIR, self.current_worksheet_name)

        # If we have a current construction layer
        if hasattr(self, 'current_layer_name') and self.current_layer_name:
            # Assuming construction layers are inside "designs" subfolder
            layer_path = os.path.join(worksheet_path, "designs", self.current_layer_name)
            if os.path.exists(layer_path):
                return layer_path
            else:
                self.message_text.append(f"Construction layer folder not found: {layer_path}")
        
        # Fallback to worksheet root
        self.message_text.append("Using worksheet root for saving (no active construction layer).")
        return worksheet_path
    
# =========================================================================================================================================================
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

# ===========================================================================================================================================================
    def format_chainage(self, x, for_dialog=False):
        """
        Convert chainage in meters to KM+Interval format.
        CRITICAL FIX: Now uses EXACT meter value for chainage string (supports sub-intervals).
        """
        if not self.zero_line_set or self.zero_start_km is None:
            return f"{x:.2f}m"

        # Total meters along the zero line
        total_meters = x

        # Base KM from zero line start
        base_km = int(self.zero_start_km)

        # Calculate total meters from KM 0+000
        total_from_zero = base_km * 1000 + total_meters

        # Final KM = integer part of total meters / 1000
        final_km = int(total_from_zero // 1000)
        chainage_m = int(round(total_from_zero % 1000))  # Exact meters within KM

        chainage_str = f"{final_km}+{chainage_m:03d}"

        # Only add approximation note if NOT for dialog (i.e., for display only)
        if not for_dialog:
            # Show exact distance in meters for transparency
            chainage_str += f" ({x:.2f}m)"

        return chainage_str

# =======================================================================================================================================
# MATERIAL LINE HANDLING
    def toggle_material_line_visibility(self, index, state):
        """Activate/deactivate a material line — drawing requires 'Start' button"""
        if 0 <= index < len(self.material_items):
            item = self.material_items[index]
            is_visible = state == Qt.Checked
            item['data']['visible'] = is_visible

            if index < len(self.material_configs):
                self.material_configs[index]['visible'] = is_visible

            if is_visible:
                self.active_material_index = index
                self.active_line_type = 'material'
                self.material_drawing_active = False

                # Clear any ongoing preview
                if hasattr(self, 'current_material_line_artist') and self.current_material_line_artist:
                    self.current_material_line_artist.remove()
                    self.current_material_line_artist = None
                self.material_drawing_points = []

                self.message_text.append(
                    f"Material line M{index+1} ACTIVATED.\n"
                    "→ Click the green 'START' button (bottom toolbar) to begin drawing.\n"
                    "→ Left-click: add points | Right-click: undo last | Click 'STOP' to finish"
                )

                # Show previously saved permanent lines
                if index in self.material_polylines_artists:
                    for artist in self.material_polylines_artists[index]:
                        artist.set_visible(True)
                        if artist not in self.ax.lines:
                            self.ax.add_line(artist)

                # Show saved filling (background + hatching)
                if index in self.material_fill_patches:
                    for patch_dict in self.material_fill_patches[index]:
                        if 'bg' in patch_dict:
                            patch_dict['bg'].set_visible(True)
                        if 'hatch' in patch_dict:
                            patch_dict['hatch'].set_visible(True)

                # Show segment labels
                if index in self.material_segment_labels:
                    for label in self.material_segment_labels[index]:
                        label.set_visible(True)

                # Show 3D actors
                if index in self.material_3d_actors:
                    for actor in self.material_3d_actors[index]:
                        actor.SetVisibility(1)  # 1 = visible

                # NEW: Load and draw saved filling if not already present
                if index not in self.material_fill_patches:
                    self.load_and_draw_material_filling(index)

                # Safe 3D render
                if hasattr(self, 'vtkWidget') and self.vtkWidget:
                    rw = self.vtkWidget.GetRenderWindow()
                    if rw:
                        rw.Render()

            else:
                # Deactivate
                if self.active_material_index == index:
                    self.active_material_index = None
                    self.active_line_type = None
                    self.material_drawing_active = False

                    # Force Stop if drawing was active
                    if self.start_stop_button.isChecked():
                        self.start_stop_button.setChecked(False)

                    # Clear preview
                    if hasattr(self, 'current_material_line_artist') and self.current_material_line_artist:
                        self.current_material_line_artist.remove()
                        self.current_material_line_artist = None

                # Hide saved lines
                if index in self.material_polylines_artists:
                    for artist in self.material_polylines_artists[index]:
                        artist.set_visible(False)

                # Hide filling patches
                if index in self.material_fill_patches:
                    for patch_dict in self.material_fill_patches[index]:
                        if 'bg' in patch_dict:
                            patch_dict['bg'].set_visible(False)
                        if 'hatch' in patch_dict:
                            patch_dict['hatch'].set_visible(False)

                # Hide segment labels
                if index in self.material_segment_labels:
                    for label in self.material_segment_labels[index]:
                        label.set_visible(False)

                # Hide 3D actors
                if index in self.material_3d_actors:
                    for actor in self.material_3d_actors[index]:
                        actor.SetVisibility(0)  # 0 = hidden

                self.message_text.append(f"Material line M{index+1} deactivated.")

                # Safe 3D render
                if hasattr(self, 'vtkWidget') and self.vtkWidget:
                    rw = self.vtkWidget.GetRenderWindow()
                    if rw:
                        rw.Render()

            # Always redraw 2D canvas
            self.canvas.draw_idle()
    # =====================================================================
    # NEW: Handle ESC key to finish material line drawing
    # =====================================================================
    def on_material_key_press(self, event):
        """Handle ESC key to finish material drawing"""
        if event.key == 'escape' and self.active_line_type == 'material':
            if len(self.material_drawing_points) >= 2:
                self.finish_material_segment()
            else:
                self.message_text.append("Need at least 2 points to finish material segment.")
    
    # =====================================================================
    def on_material_draw_click(self, event):
        """Handle mouse clicks only when material drawing is active"""
        if not self.material_drawing_active or event.inaxes != self.ax:
            return

        x, y = event.xdata, event.ydata
        if x is None or y is None or not (0 <= x <= self.total_distance):
            return

        idx = self.active_material_index

        if event.button == 1:  # Left click — add point
            self.material_drawing_points.append((x, y))

            # Update preview
            if hasattr(self, 'current_material_line_artist') and self.current_material_line_artist:
                self.current_material_line_artist.remove()

            xs = [p[0] for p in self.material_drawing_points]
            ys = [p[1] for p in self.material_drawing_points]
            self.current_material_line_artist = self.ax.plot(
                xs, ys, color='orange', linewidth=3, linestyle='--', alpha=0.7
            )[0]

            # Add segment label if ≥2 points
            if len(self.material_drawing_points) >= 2:
                prev_x, prev_y = self.material_drawing_points[-2]
                mid_x = (prev_x + x) / 2
                mid_y = max(prev_y, y) + 0.2
                seg_num = len(self.material_drawing_points) - 1
                label_text = f"M{idx+1}-{seg_num}"

                annot = self.ax.annotate(
                    label_text, (mid_x, mid_y),
                    xytext=(0, 10),
                    textcoords='offset points',
                    ha='center',
                    fontsize=9,
                    fontweight='bold',
                    color='white',
                    bbox=dict(boxstyle='round,pad=0.4', facecolor='orange', alpha=0.9, edgecolor='darkorange'),
                    picker=10  # Fix: Use 'picker=10' for pick radius (replaces any 'pickradius')
                )

                annot.point_data = {
                    'type': 'material_segment_label',
                    'material_index': idx,
                    'segment_number': seg_num,
                    'from_chainage_m': prev_x,
                    'to_chainage_m': x,
                    'config': {}
                }

                if idx not in self.material_segment_labels:
                    self.material_segment_labels[idx] = []
                self.material_segment_labels[idx].append(annot)

            self.message_text.append(f"Point added at {self.format_chainage(x)}")
            self.canvas.draw_idle()

        elif event.button == 3:  # Right click — undo
            if self.material_drawing_points:
                self.material_drawing_points.pop()

                # Remove last label
                if idx in self.material_segment_labels and self.material_segment_labels[idx]:
                    last_label = self.material_segment_labels[idx].pop()
                    last_label.remove()

                # Redraw preview
                if hasattr(self, 'current_material_line_artist') and self.current_material_line_artist:
                    self.current_material_line_artist.remove()

                if self.material_drawing_points:
                    xs = [p[0] for p in self.material_drawing_points]
                    ys = [p[1] for p in self.material_drawing_points]
                    self.current_material_line_artist = self.ax.plot(
                        xs, ys, color='orange', linewidth=3, linestyle='--', alpha=0.7
                    )[0]

                self.canvas.draw_idle()
                self.message_text.append("Last point removed.")

# ==============================================================================================================================================================
    def _get_material_line_points_for_segment(self, material_index, from_m, to_m):
        """
        UPDATED & 100% WORKING: Handles BOTH old single-segment JSONs
        AND new multi-segment JSONs (with "segments" list).
        Combines all polyline_points from every segment for full accurate hatching.
        """
        if not hasattr(self, 'material_items') or not self.material_items:
            self.message_text.append("No material lines defined yet.")
            return [], []
        if material_index >= len(self.material_items):
            self.message_text.append(f"Material index {material_index} out of range.")
            return [], []
        target_item = self.material_items[material_index]
        folder_name = target_item.get('folder') or target_item.get('data', {}).get('folder_name')
        if not folder_name:
            self.message_text.append("Material folder name not found.")
            return [], []
        json_path = os.path.join(self.current_construction_layer_path, f"{folder_name}.json")
        if not os.path.exists(json_path):
            self.message_text.append(f"Material JSON not found: {json_path}")
            return [], []
        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                seg_data = json.load(f)
            # === CRITICAL FIX: Extract polyline points correctly ===
            all_poly_points = []
            if "segments" in seg_data and seg_data["segments"]:
                # NEW FORMAT: collect points from ALL segments
                for segment in seg_data["segments"]:
                    points = segment.get("polyline_points", [])
                    all_poly_points.extend(points)
                self.message_text.append(f"Loaded multi-segment JSON: {len(seg_data['segments'])} segments → {len(all_poly_points)} total points")
            elif "polyline_points" in seg_data:
                # OLD FORMAT: direct at root
                all_poly_points = seg_data.get("polyline_points", [])
                self.message_text.append("Loaded old single-segment JSON format.")
            else:
                self.message_text.append("No polyline_points found in JSON (neither in segments nor root).")
                return [], []
            if len(all_poly_points) < 2:
                self.message_text.append("Not enough polyline points saved to draw hatching.")
                return [], []
            # Sort by chainage to be safe
            all_poly_points.sort(key=lambda p: p["chainage_m"])
            mat_xs = [p["chainage_m"] for p in all_poly_points]
            mat_ys = [p["relative_elevation_m"] for p in all_poly_points]
        except Exception as e:
            self.message_text.append(f"Error reading material JSON: {str(e)}")
            return [], []
        # Generate dense line for smooth hatching
        import numpy as np
        x_dense = np.linspace(from_m, to_m, 1500)
        y_dense = np.interp(x_dense, mat_xs, mat_ys, left=mat_ys[0], right=mat_ys[-1])
        self.message_text.append(f"✓ Hatching drawn using {len(mat_xs)} saved points")
        return x_dense.tolist(), y_dense.tolist()

# =======================================================================================================================================
    def finish_material_segment(self):
        """Finish the current material line drawing and save permanently with real-world coordinates"""
        material_idx = self.active_material_index

        if not self.material_drawing_points or len(self.material_drawing_points) < 2:
            self.message_text.append("Not enough points to finish segment.")
            return

        first_point = self.material_drawing_points[0]
        last_point  = self.material_drawing_points[-1]

        first_x = first_point[0]
        last_x  = last_point[0]

        from_chainage_str = self.format_chainage(first_x, for_dialog=True)
        to_chainage_str   = self.format_chainage(last_x, for_dialog=True)

        dialog = MaterialSegmentDialog(
            from_chainage=from_chainage_str,
            to_chainage=to_chainage_str,
            material_thickness=None,
            width=None,
            after_rolling=None,
            parent=self
        )
        dialog.setWindowTitle(f"Material Segment - M{material_idx+1}")

        if dialog.exec_() == QDialog.Accepted:
            config = dialog.get_data()
            if config is None:
                return

            from_m = config.get('from_chainage_m')
            to_m   = config.get('to_chainage_m')
            thickness_m = config.get('material_thickness_m', 0.0)

            if from_m is None or to_m is None or to_m <= from_m:
                QMessageBox.warning(self, "Invalid Input", "From chainage must be less than To chainage.")
                return

            if thickness_m <= 0:
                reply = QMessageBox.question(
                    self, "Thickness Zero",
                    "Material thickness is 0 m – this is for documentation only.\nContinue anyway?",
                    QMessageBox.Yes | QMessageBox.No
                )
                if reply == QMessageBox.No:
                    return

            # ===================================================
            #   Prepare polyline_points WITH absolute coordinates
            # ===================================================
            polyline_points = []
            for p in self.material_drawing_points:
                chainage = round(p[0], 3)
                rel_elev = round(p[1], 3)

                X, Y, Z_center = self.get_real_coordinates_from_chainage(chainage)
                if X is None or Y is None or Z_center is None:
                    X, Y, Z_center = self.interpolate_xyz(chainage)

                abs_z = Z_center + rel_elev

                polyline_points.append({
                    "chainage_m": chainage,
                    "relative_elevation_m": rel_elev,
                    "absolute_coordinates": [
                        round(X, 3),
                        round(Y, 3),
                        round(abs_z, 3)
                    ]
                })

            # Save
            self.save_material_segment_to_json(
                material_idx=material_idx,
                config=config,
                from_m=from_m,
                to_m=to_m,
                point_number=None,
                polyline_points=polyline_points
            )

            # Show volume feedback (optional but recommended)
            try:
                json_path = os.path.join(self.current_construction_layer_path,
                                    f"{self.material_configs[material_idx]['folder_name']}.json")
                with open(json_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                last_seg = data["segments"][-1]
                self.message_text.append(
                    f"Segment saved → Volume: {last_seg.get('volume_m3', 0):.2f} m³   "
                    f"Avg height: {last_seg.get('avg_height_m', 0):.3f} m"
                )
            except Exception as ex:
                self.message_text.append(f"Note: Could not read saved volume info ({ex})")

            # Draw permanent line
            xs = [p[0] for p in self.material_drawing_points]
            ys = [p[1] for p in self.material_drawing_points]

            permanent_line = self.ax.plot(
                xs, ys,
                color='orange', linewidth=3, linestyle='-',
                marker='o', markersize=6,
                markerfacecolor='orange', markeredgecolor='darkred',
                alpha=0.9
            )[0]

            if material_idx not in self.material_polylines_artists:
                self.material_polylines_artists[material_idx] = []
            self.material_polylines_artists[material_idx].append(permanent_line)

            if hasattr(self, 'current_material_line_artist') and self.current_material_line_artist:
                self.current_material_line_artist.remove()
                self.current_material_line_artist = None

            self.material_drawing_points = []

            # Draw filling
            self.draw_material_filling(
                from_chainage_m=from_m,
                to_chainage_m=to_m,
                thickness_m=thickness_m,
                material_index=material_idx,
                material_config=self.material_configs[material_idx],
                color='#FF9800',
                alpha=0.6,
                width_m=config.get('width_m', 0.0)
            )

            self.message_text.append(f"Material segment M{material_idx+1} finished and filled.")
            self.message_text.append(f"   Chainage: {self.format_chainage(from_m)} → {self.format_chainage(to_m)}")
            self.message_text.append(f"   Nominal thickness (doc only): {thickness_m*1000:.0f} mm")

            self.canvas.draw_idle()

        else:
            # Cancelled
            if hasattr(self, 'current_material_line_artist') and self.current_material_line_artist:
                self.current_material_line_artist.remove()
                self.current_material_line_artist = None
            self.material_drawing_points = []
            self.message_text.append("Material segment cancelled.")
            self.canvas.draw_idle()

# =======================================================================================================================================
    def on_material_drawing_toggle(self, checked):
        """Handle clicks on Start/Stop button"""
        if self.active_material_index is None:
            if checked:
                self.start_stop_button.setChecked(False)
                self.message_text.append("⚠️ Please first check a Material Line box to select it.")
            return

        if checked:  # START
            self.material_drawing_active = True
            self.material_drawing_points = []

            # Clear old preview
            if hasattr(self, 'current_material_line_artist') and self.current_material_line_artist:
                self.current_material_line_artist.remove()
                self.current_material_line_artist = None

            # Clear old labels for this material (start fresh)
            idx = self.active_material_index
            if idx in self.material_segment_labels:
                for label in self.material_segment_labels[idx]:
                    label.remove()
                self.material_segment_labels[idx] = []

            self.message_text.append(
                f"🟢 STARTED drawing Material M{idx+1}\n"
                "• Left-click on graph to add points\n"
                "• Right-click to remove last point\n"
                "• Click 'STOP' when complete → dialog will open to configure the full segment"
            )

        else:  # STOP
            self.material_drawing_active = False
            self.finish_material_line_drawing()  # This now triggers dialog + full segment save

# ================================================================================================================================================================
    
    def finish_material_line_drawing(self):
        """Called when STOP is clicked - finish multi-point material line drawing"""
        if len(self.material_drawing_points) < 2:
            if hasattr(self, 'current_material_line_artist') and self.current_material_line_artist:
                self.current_material_line_artist.remove()
                self.current_material_line_artist = None
            self.material_drawing_points = []
            idx = self.active_material_index
            if idx in self.material_segment_labels:
                for label in self.material_segment_labels[idx]:
                    label.remove()
                self.material_segment_labels[idx] = []
            self.message_text.append("Drawing cancelled — need at least 2 points.")
            self.canvas.draw_idle()
            return

        idx = self.active_material_index

        # Sort + snap to intervals
        self.material_drawing_points.sort(key=lambda p: p[0])
        drawn_xs = [p[0] for p in self.material_drawing_points]
        drawn_ys = [p[1] for p in self.material_drawing_points]

        min_x = drawn_xs[0]
        max_x = drawn_xs[-1]

        import numpy as np
        interval = getattr(self, 'zero_interval', 20.0)
        first_target = np.ceil(min_x / interval) * interval
        target_xs = np.arange(first_target, max_x + 1e-6, interval).tolist()

        all_xs = sorted(set([min_x, max_x] + target_xs))
        all_ys = np.interp(all_xs, drawn_xs, drawn_ys, left=drawn_ys[0], right=drawn_ys[-1])

        self.material_drawing_points = list(zip(all_xs, all_ys))

        # Clean old permanent lines
        if idx in self.material_polylines_artists:
            for line in self.material_polylines_artists[idx]:
                if line in self.ax.lines:
                    line.remove()
            self.material_polylines_artists[idx] = []

        # Draw permanent line
        xs = [p[0] for p in self.material_drawing_points]
        ys = [p[1] for p in self.material_drawing_points]

        permanent_line = self.ax.plot(
            xs, ys,
            color='orange', linewidth=3, linestyle='-',
            marker='o', markersize=6,
            markerfacecolor='orange', markeredgecolor='darkred',
            alpha=0.9
        )[0]

        self.material_polylines_artists.setdefault(idx, []).append(permanent_line)

        # Clear old labels
        if idx in self.material_segment_labels:
            for label in self.material_segment_labels[idx]:
                label.remove()
            self.material_segment_labels[idx] = []

        # Add labels between points
        self.material_segment_labels[idx] = []
        num_points = len(self.material_drawing_points)

        for i in range(num_points - 1):
            prev_x, prev_y = self.material_drawing_points[i]
            curr_x, curr_y = self.material_drawing_points[i + 1]
            mid_x = (prev_x + curr_x) / 2
            mid_y = max(prev_y, curr_y) + 0.25
            seg_num = i + 1
            label_text = f"M{idx+1}-{seg_num}"

            annot = self.ax.annotate(
                label_text, (mid_x, mid_y),
                xytext=(0, 12), textcoords='offset points',
                ha='center', va='bottom',
                fontsize=9, fontweight='bold', color='white',
                bbox=dict(boxstyle='round,pad=0.4', facecolor='orange', alpha=0.9, edgecolor='darkorange'),
                picker=10
            )

            annot.point_data = {
                'type': 'material_segment_label',
                'material_index': idx,
                'segment_number': seg_num,
                'from_chainage_m': prev_x,
                'to_chainage_m': curr_x,
                'config': {}
            }
            self.material_segment_labels[idx].append(annot)

        # Clear preview
        if hasattr(self, 'current_material_line_artist') and self.current_material_line_artist:
            self.current_material_line_artist.remove()
            self.current_material_line_artist = None

        self.canvas.draw_idle()

        # Open configuration dialog
        from_chainage_str = self.format_chainage(min_x, for_dialog=True)
        to_chainage_str = self.format_chainage(max_x, for_dialog=True)

        dialog = MaterialSegmentDialog(
            from_chainage=from_chainage_str,
            to_chainage=to_chainage_str,
            material_thickness=None,
            width=None,
            after_rolling=None,
            parent=self
        )
        dialog.setWindowTitle(f"Configure Material Line - M{idx+1}")

        if dialog.exec_() == QDialog.Accepted:
            config = dialog.get_data()
            if config is None:
                self.message_text.append("Configuration cancelled.")
                self.canvas.draw_idle()
                return

            from_m = config.get('from_chainage_m')
            to_m = config.get('to_chainage_m')
            thickness_m = config.get('material_thickness_m', 0.0)
            width_m = config.get('width_m', 20.0)  # fallback
            after_rolling_m = config.get('after_rolling_thickness_m', 0.0)

            if from_m is None or to_m is None or to_m <= from_m:
                QMessageBox.warning(self, "Invalid Range", "To chainage must be greater than From chainage.")
                return

            # Load reference baseline
            ref_xs, ref_ys = self._load_baseline_from_design_layer(self.material_configs[idx])

            if not ref_xs or len(ref_xs) < 2:
                self.message_text.append("Warning: No reference baseline found → using 0.0 as bottom")
                ref_xs = all_xs
                ref_ys = [0.0] * len(all_xs)
            else:
                self.message_text.append(f"Reference baseline loaded successfully with {len(ref_xs)} points")

            # Build segments with REAL thickness calculation
            segments = []
            for i in range(num_points - 1):
                seg_from_m = self.material_drawing_points[i][0]
                seg_to_m = self.material_drawing_points[i + 1][0]
                seg_num = i + 1

                from_X, from_Y, from_Z = self.interpolate_xyz(seg_from_m)
                to_X, to_Y, to_Z = self.interpolate_xyz(seg_to_m)

                seg_poly_points = []
                for j in range(num_points):
                    x, y = self.material_drawing_points[j]
                    if seg_from_m <= x <= seg_to_m + 1e-6:
                        chainage = round(x, 3)
                        material_rel = round(y, 3)

                        # Get reference elevation at this chainage
                        ref_elev = np.interp(
                            chainage,
                            ref_xs,
                            ref_ys,
                            left=ref_ys[0],
                            right=ref_ys[-1]
                        )

                        # Calculate real thickness (material - reference)
                        real_thickness = material_rel - ref_elev

                        # Positive thickness for volume calculation (filling)
                        thickness_positive = max(0.0, real_thickness)  # Changed to prefer positive fill

                        # Get absolute coordinates
                        X, Y, Z_center = self.get_real_coordinates_from_chainage(chainage)
                        if X is None or Y is None or Z_center is None:
                            X, Y, Z_center = self.interpolate_xyz(chainage)
                        abs_z = Z_center + material_rel

                        seg_poly_points.append({
                            "chainage_m": chainage,
                            "relative_elevation_m": material_rel,
                            "reference_elevation_m": round(ref_elev, 3),
                            "actual_thickness_m": round(real_thickness, 3),
                            "thickness_positive_m": round(thickness_positive, 3),
                            "absolute_coordinates": [
                                round(X, 3),
                                round(Y, 3),
                                round(abs_z, 3)
                            ]
                        })

                segments.append({
                    "segment_number": seg_num,
                    "segment_label": f"M{idx+1}-{seg_num}",
                    "from_chainage_m": round(seg_from_m, 3),
                    "to_chainage_m": round(seg_to_m, 3),
                    "from_chainage_str": self.format_chainage(seg_from_m, for_dialog=True),
                    "to_chainage_str": self.format_chainage(seg_to_m, for_dialog=True),
                    "from_coordinates": [round(from_X, 3), round(from_Y, 3), round(from_Z, 3)],
                    "to_coordinates": [round(to_X, 3), round(to_Y, 3), round(to_Z, 3)],
                    "polyline_points": seg_poly_points,
                    "material_thickness_m": thickness_m,
                    "width_m": width_m,
                    "after_rolling_thickness_m": after_rolling_m,
                    "material_description": config.get('material_description', '')
                })

                # Debug: show average thickness for this segment
                thicknesses = [p["thickness_positive_m"] for p in seg_poly_points]
                avg_seg = np.mean(thicknesses) if thicknesses else 0.0
                self.message_text.append(f"Segment {seg_num}: avg positive thickness = {avg_seg:.3f} m")

            # Save with calculated thicknesses
            self.save_material_segment_to_json(
                material_idx=idx,
                config=config,
                from_m=from_m,
                to_m=to_m,
                segments_list=segments
            )

            self.draw_material_filling(
                from_chainage_m=from_m,
                to_chainage_m=to_m,
                thickness_m=thickness_m,
                material_index=idx,
                material_config=self.material_configs[idx],
                color='#FF9800',
                alpha=0.6,
                width_m=width_m
            )

            total_segs = len(segments)
            self.message_text.append(
                f"Material M{idx+1} saved successfully!\n"
                f"→ {total_segs} segments created with REAL height calculation"
            )
        else:
            self.message_text.append("Configuration cancelled.")

        self.canvas.draw_idle()
        self.material_drawing_points = []

# =======================================================================================================================================
# CONSOLIDATED: Single setup_label_click_handler (remove duplicates; use unified on_label_pick)
# =======================================================================================================================================
    def setup_label_click_handler(self):
        """Set up the unified label click event handler after canvas is fully initialized"""
        if self.canvas:
            # Disconnect any existing to avoid duplicates
            if hasattr(self, 'label_pick_id') and self.label_pick_id is not None:
                try:
                    self.canvas.mpl_disconnect(self.label_pick_id)
                except ValueError:
                    pass  # Already disconnected
            self.label_pick_id = self.canvas.mpl_connect('pick_event', self.on_label_pick)  # Unified handler
            # print("Unified pick event connected")  # Debug - remove later

# ====================================================================================================================================================
    def on_label_pick(self, event):
        """
        Unified handler for clicking on labels (construction dots or material dots).
        Handles configuration dialogs and updates visuals accordingly.
        Also saves material segment data to JSON files.
        """
        if not hasattr(event.artist, 'point_data'):
            return

        artist = event.artist
        point_data = artist.point_data

        # === MATERIAL SEGMENT LABEL CLICKED ===
        if point_data.get('type') == 'material_segment_label':
            self.on_material_segment_label_clicked(event.artist)
            return

        # =====================================================================
        # Handle CONSTRUCTION dot click
        # =====================================================================
        if point_data.get('type') == 'construction_dot':
            point_number = point_data['point_number']
            x = point_data['x']
            y = point_data['y']

            # Calculate chainage label (KM+Interval format)
            if self.zero_line_set and hasattr(self, 'zero_start_km') and self.zero_start_km is not None:
                interval_number = int(round(x / self.zero_interval)) if self.zero_interval > 0 else 0
                interval_value = interval_number * self.zero_interval
                chainage_label = f"{self.zero_start_km}+{interval_value:03d}"
                if abs(x - interval_value) > 0.01:
                    chainage_label += f" (approx, actual: {x:.2f}m)"
            else:
                chainage_label = f"Distance: {x:.2f}m"

            # Open construction configuration dialog
            dialog = ConstructionConfigDialog(chainage_label, self)
            dialog.setWindowTitle(f"Construction Configuration - Point P{point_number}")

            if dialog.exec_() == QDialog.Accepted:
                config = dialog.get_configuration()
                self.message_text.append(f"Construction configuration saved for Point P{point_number}:")
                self.message_text.append(f"  Chainage: {config['chainage']}")

                # Store config
                point_data['config'] = config

                # Update label appearance to show configured
                if hasattr(artist, 'set_text'):
                    artist.set_text(f'P{point_number}✓')
                if hasattr(artist, 'set_bbox'):
                    artist.set_bbox(dict(
                        boxstyle='round,pad=0.5',
                        facecolor='lightgreen',
                        alpha=0.9,
                        edgecolor='green',
                        linewidth=2
                    ))

                self.canvas.draw_idle()

            return

        # =====================================================================
        # Handle MATERIAL dot click
        # =====================================================================
        elif point_data.get('type') == 'material_dot':
            point_number = point_data['point_number']
            x = point_data['x']                     # chainage in meters
            y = point_data['y']
            material_idx = point_data['material_index']
            current_config = point_data.get('config', {})

            # Pre-fill "From" with the clicked point's chainage
            from_chainage_str = self.format_chainage(x, for_dialog=True)

            # Open Material Segment Dialog
            dialog = MaterialSegmentDialog(
                from_chainage=from_chainage_str,
                to_chainage="",
                material_thickness=current_config.get('material_thickness_m'),
                width=current_config.get('width_m'),
                after_rolling=current_config.get('after_rolling_thickness_m'),
                material_description=current_config.get('material_description'),
                parent=self
            )
            dialog.setWindowTitle(f"Material Segment Configuration - M{material_idx + 1}-{point_number}")

            if dialog.exec_() == QDialog.Accepted:
                config = dialog.get_data()
                if config is None:
                    return

                from_m = config['from_chainage_m']
                to_m   = config['to_chainage_m']
                thickness_m = config['material_thickness_m']

                # Validation
                if from_m is None or to_m is None:
                    QMessageBox.warning(self, "Invalid Input", "Both From and To chainage must be provided.")
                    return
                if to_m <= from_m:
                    QMessageBox.warning(self, "Invalid Range", "To chainage must be greater than From chainage.")
                    return
                if thickness_m <= 0:
                    QMessageBox.warning(self, "Invalid Thickness", "Thickness must be greater than 0 mm.")
                    return

                # ========================================================
                # SAVE TO JSON FILE (in materials/ folder)
                # ========================================================
                self.save_material_segment_to_json(
                    material_idx=material_idx,
                    config=config,
                    from_m=from_m,
                    to_m=to_m,
                    point_number=point_number
                )

                # Update stored config on the point
                point_data['config'] = config

                # Update label text and visual feedback
                new_label = f"M{material_idx + 1}-{point_number}✓"
                if thickness_m:
                    new_label += f" ({thickness_m:.0f}mm)"
                artist.set_text(new_label)

                artist.set_bbox(dict(
                    facecolor='lightgreen',
                    alpha=0.9,
                    edgecolor='green',
                    linewidth=2
                ))

                # Draw the material filling (adds new one — previous ones stay intact)
                self.draw_material_filling(
                    from_chainage_m=from_m,
                    to_chainage_m=to_m,
                    thickness_m=thickness_m,
                    material_index=material_idx,
                    material_config=self.material_configs[material_idx],
                    color='#FF9800',
                    alpha=0.55,
                    width_m=config.get('width_m', 0.0)  # NEW: Pass width for 3D
                )

                # Optional: Store segment for later export or editing
                if not hasattr(self, 'material_segments'):
                    self.material_segments = []
                self.material_segments.append({
                    'material_index': material_idx,
                    'point_number': point_number,
                    'from_m': from_m,
                    'to_m': to_m,
                    'thickness_m': thickness_m,
                    'width_m': config.get('width_m'),
                    'after_rolling_mm': config.get('after_rolling_thickness_m'),
                    'label_artist': artist
                })

                self.canvas.draw_idle()

                # User feedback
                self.message_text.append(f"Material segment configured: M{material_idx + 1}-{point_number}")
                self.message_text.append(f"  From: {self.format_chainage(from_m)}")
                self.message_text.append(f"  To:   {self.format_chainage(to_m)}")
                self.message_text.append(f"  Thickness: {thickness_m:.0f} mm")
                self.message_text.append(f"  Width: {config.get('width_m', 0):.1f} m")
                self.message_text.append(f"  After Rolling: {config.get('after_rolling_thickness_m', 0):.0f} mm")
                self.message_text.append("  → JSON file saved in 'materials/' folder")
                self.message_text.append("")

            else:
                self.message_text.append(f"Material segment configuration cancelled for M{material_idx + 1}-{point_number}")

# ===============================================================================================================================================================
    def on_material_segment_label_clicked(self, artist):
        """Handle click on segment label → update values in existing JSON"""
        point_data = artist.point_data
        mat_idx = point_data['material_index']
        seg_num = point_data['segment_number']
        from_m = point_data['from_chainage_m']
        to_m = point_data['to_chainage_m']

        from_str = self.format_chainage(from_m, for_dialog=True)
        to_str = self.format_chainage(to_m, for_dialog=True)

        # Find the JSON file
        folder_name = self.material_configs[mat_idx].get('folder_name')
        json_path = os.path.join(self.current_construction_layer_path, f"{folder_name}.json")
        if not os.path.exists(json_path):
            self.message_text.append(f"Material JSON not found: {json_path}")
            return

        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            self.message_text.append(f"Error reading JSON: {str(e)}")
            return

        # Find the segment
        target_segment = None
        for seg in data.get("segments", []):
            if seg.get("segment_number") == seg_num:
                target_segment = seg
                break

        if not target_segment:
            self.message_text.append("Segment not found in JSON.")
            return

        # Open dialog with current values
        dialog = MaterialSegmentDialog(
            from_chainage=from_str,
            to_chainage=to_str,
            material_thickness=target_segment.get("material_thickness_m", 0.0),
            width=target_segment.get("width_m"),
            after_rolling=target_segment.get("after_rolling_thickness_m"),
            material_description=target_segment.get("material_description"),
            parent=self
        )
        dialog.setWindowTitle(f"Update Segment M{mat_idx + 1}-{seg_num}")

        if dialog.exec_() == QDialog.Accepted:
            config = dialog.get_data()
            if config is None:
                return

            target_segment["material_thickness_m"] = config.get('material_thickness_m', target_segment["material_thickness_m"])
            target_segment["width_m"] = config.get('width_m', target_segment["width_m"])
            target_segment["after_rolling_thickness_m"] = config.get('after_rolling_thickness_m', target_segment["after_rolling_thickness_m"])
            target_segment["material_description"] = config.get('material_description', target_segment.get("material_description", ""))

            # Save updated JSON
            try:
                with open(json_path, 'w', encoding='utf-8') as f:
                    json.dump(data, f, indent=4, ensure_ascii=False)

                # Update label
                thickness = target_segment["material_thickness_m"]
                new_text = f"M{mat_idx + 1}-{seg_num}"
                if thickness > 0:
                    new_text += f" ({thickness*1000:.0f}mm)"
                artist.set_text(new_text)
                artist.set_bbox(dict(boxstyle='round,pad=0.5', facecolor='lightgreen', alpha=0.9, edgecolor='green', linewidth=2))

                self.canvas.draw_idle()

                # Redraw filling (using overall extents)
                overall_from = data["overall_from_chainage"]["chainage_m"]
                overall_to = data["overall_to_chainage"]["chainage_m"]
                self.draw_material_filling(
                    from_chainage_m=overall_from,
                    to_chainage_m=overall_to,
                    thickness_m=thickness,  # use updated value
                    material_index=mat_idx,
                    material_config=self.material_configs[mat_idx],
                    color='#FF9800',
                    alpha=0.6,
                    width_m=target_segment["width_m"]  # NEW: Use updated width
                )

                self.message_text.append(f"Segment M{mat_idx + 1}-{seg_num} updated and saved.")

            except Exception as e:
                QMessageBox.critical(self, "Save Failed", f"Could not update JSON:\n{str(e)}")
        else:
            self.message_text.append("Update cancelled.")

# ===========================================================================================================================================================
    def interpolate_xyz(self, chainage_m):
        """
        Kept for backward compatibility, but now prioritizes accurate alignment.
        Falls back to linear only if alignment is not available.
        """
        X, Y, Z = self.get_real_coordinates_from_chainage(chainage_m)
        if X is not None and Y is not None and Z is not None:
            return round(X, 3), round(Y, 3), round(Z, 3)

        # Fallback: linear interpolation using zero line start/end (only if alignment missing)
        if (not self.zero_line_set or
            not hasattr(self, 'zero_start_point') or
            not hasattr(self, 'zero_end_point') or
            not hasattr(self, 'total_distance') or
            self.total_distance <= 0):
            return 0.0, 0.0, 0.0

        import numpy as np
        start_pt = np.array(self.zero_start_point)
        end_pt = np.array(self.zero_end_point)
        total_len = float(self.total_distance)
        t = np.clip(chainage_m / total_len, 0.0, 1.0)
        point_3d = start_pt + t * (end_pt - start_pt)
        return round(float(point_3d[0]), 3), round(float(point_3d[1]), 3), round(float(point_3d[2]), 3)
    
# =================================================================================================================================================================
    def save_material_segment_to_json(self, material_idx, config, from_m, to_m, point_number=None, polyline_points=None, segments_list=None):
        """Save material line JSON with volume & height statistics."""
        import os
        import json
        from datetime import datetime
        import numpy as np

        mat_config = self.material_configs[material_idx]
        folder_name = mat_config.get('folder_name')
        display_name = mat_config.get('name', folder_name)
        if not folder_name:
            QMessageBox.critical(self, "Error", "Material folder name not found.")
            return

        filepath = os.path.join(self.current_construction_layer_path, f"{folder_name}.json")

        from_X, from_Y, from_Z = self.get_real_coordinates_from_chainage(from_m)
        to_X, to_Y, to_Z = self.get_real_coordinates_from_chainage(to_m)

        if from_X is None or to_X is None:
            from_X, from_Y, from_Z = self.interpolate_xyz(from_m)
            to_X, to_Y, to_Z = self.interpolate_xyz(to_m)

        from_chainage_str = self.format_chainage(from_m, for_dialog=True)
        to_chainage_str = self.format_chainage(to_m, for_dialog=True)

        # Handle single segment → multi-segment conversion
        if segments_list is None and polyline_points is not None:
            thickness_m = config.get('material_thickness_m', 0.0)
            width_m = config.get('width_m', 20.0)
            after_rolling_m = config.get('after_rolling_thickness_m', 0.0)
            seg_poly_points = []
            for p in polyline_points:
                chainage = p["chainage_m"]
                rel_elev = p["relative_elevation_m"]
                abs_X, abs_Y, abs_Z = self.get_real_coordinates_from_chainage(chainage)
                if abs_X is None:
                    abs_X, abs_Y, abs_Z = self.interpolate_xyz(chainage)
                seg_poly_points.append({
                    "chainage_m": round(chainage, 3),
                    "relative_elevation_m": round(rel_elev, 3),
                    "absolute_coordinates": [round(abs_X, 3), round(abs_Y, 3), round(abs_Z + rel_elev, 3)]
                })
            segments_list = [{
                "segment_number": 1 if point_number is None else point_number,
                "segment_label": f"M{material_idx+1}-{1 if point_number is None else point_number}",
                "from_chainage_m": round(from_m, 3),
                "to_chainage_m": round(to_m, 3),
                "from_chainage_str": from_chainage_str,
                "to_chainage_str": to_chainage_str,
                "from_coordinates": [round(from_X, 3), round(from_Y, 3), round(from_Z, 3)],
                "to_coordinates": [round(to_X, 3), round(to_Y, 3), round(to_Z, 3)],
                "polyline_points": seg_poly_points,
                "material_thickness_m": thickness_m,
                "width_m": width_m,
                "after_rolling_thickness_m": after_rolling_m,
                "material_description": config.get('material_description', '')
            }]

        # Ensure absolute coordinates in all points while preserving existing fields
        if segments_list:
            for segment in segments_list:
                updated_points = []
                for p in segment.get("polyline_points", []):
                    chainage = p["chainage_m"]
                    rel_elev = p["relative_elevation_m"]
                    abs_X, abs_Y, abs_Z = self.get_real_coordinates_from_chainage(chainage)
                    if abs_X is None:
                        abs_X, abs_Y, abs_Z = self.interpolate_xyz(chainage)
                    abs_Z_total = abs_Z + rel_elev if abs_Z is not None else rel_elev

                    # Preserve all existing fields and only update/add absolute_coordinates
                    updated_p = p.copy()  # Copy original dict to keep all fields like thickness
                    updated_p["absolute_coordinates"] = [round(abs_X, 3), round(abs_Y, 3), round(abs_Z_total, 3)]

                    updated_points.append(updated_p)
                segment["polyline_points"] = updated_points
                # Update from/to coordinates
                fX, fY, fZ = self.get_real_coordinates_from_chainage(segment["from_chainage_m"])
                tX, tY, tZ = self.get_real_coordinates_from_chainage(segment["to_chainage_m"])
                if fX is None: fX, fY, fZ = self.interpolate_xyz(segment["from_chainage_m"])
                if tX is None: tX, tY, tZ = self.interpolate_xyz(segment["to_chainage_m"])
                segment["from_coordinates"] = [round(fX, 3), round(fY, 3), round(fZ, 3)]
                segment["to_coordinates"] = [round(tX, 3), round(tY, 3), round(tZ, 3)]

        # ── Calculate volume and heights for each segment ──
        total_volume = 0.0
        for segment in segments_list:
            width_m = segment.get("width_m", 20.0)
            if width_m <= 0.01:
                segment["volume_m3"] = 0.0
                segment["avg_height_m"] = 0.0
                segment["max_height_m"] = 0.0
                segment["min_height_m"] = 0.0
                continue

            points = segment.get("polyline_points", [])
            if len(points) < 2:
                segment["volume_m3"] = 0.0
                continue

            # Make sure points are sorted by chainage
            points = sorted(points, key=lambda p: p["chainage_m"])

            chainages = np.array([p["chainage_m"] for p in points])

            # ── Use positive thickness ──
            heights = np.array([p.get("thickness_positive_m", 0.0) for p in points])

            if len(heights) == 0 or np.all(heights == 0):
                segment["volume_m3"] = 0.0
                segment["avg_height_m"] = 0.0
                segment["max_height_m"] = 0.0
                segment["min_height_m"] = 0.0
                self.message_text.append("All heights are 0.0 → volume 0 for segment")
                continue

            avg_height = float(np.mean(heights))
            max_height = float(np.max(heights))
            min_height = float(np.min(heights))

            # Trapezoidal volume integration
            volume = 0.0
            for i in range(len(chainages) - 1):
                ds = chainages[i + 1] - chainages[i]
                if ds <= 0:
                    continue
                h1 = heights[i]
                h2 = heights[i + 1]
                h_avg = (h1 + h2) / 2.0
                dV = width_m * h_avg * ds
                volume += dV

            segment["volume_m3"] = round(volume, 2)
            segment["avg_height_m"] = round(avg_height, 3)
            segment["max_height_m"] = round(max_height, 3)
            segment["min_height_m"] = round(min_height, 3)

            total_volume += volume

        data = {
            "material_line_folder": folder_name,
            "material_line_name": display_name,
            "material_line_id": f"M{material_idx+1}",
            "overall_from_chainage": {
                "coordinates": [round(from_X, 3), round(from_Y, 3), round(from_Z, 3)],
                "chainage_m": round(from_m, 3),
                "chainage_str": from_chainage_str
            },
            "overall_to_chainage": {
                "coordinates": [round(to_X, 3), round(to_Y, 3), round(to_Z, 3)],
                "chainage_m": round(to_m, 3),
                "chainage_str": to_chainage_str
            },
            "total_segments": len(segments_list) if segments_list else 0,
            "segments": segments_list or [],
            "created_at": datetime.now().isoformat(),
            "worksheet": self.current_worksheet_name,
            "construction_layer": os.path.basename(self.current_construction_layer_path),
            "total_volume_m3": round(total_volume, 2)
        }
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=4, ensure_ascii=False)
            self.message_text.append(f"JSON saved: {os.path.basename(filepath)}")
            # Now update the single config file in construction layer root
            self.update_material_lines_config()
        except Exception as e:
            QMessageBox.critical(self, "Save Failed", f"Error saving JSON:\n{str(e)}")


# =====================================================================================================================================
    def update_material_lines_config(self):
        """Update the single material_lines_config.txt in the construction layer root with all material lines."""
        import json
        from datetime import datetime  # Make sure datetime is imported at top or here

        # FIXED PATH: Save directly in construction layer root, NOT in any material subfolder
        config_path = os.path.join(self.current_construction_layer_path, "material_lines_config.txt")

        data = {
            "worksheet_name": self.current_worksheet_name,
            "project_name": getattr(self, 'project_name', "Project_1"),
            "created_by": getattr(self, 'current_user', "admin123"),
            # FIXED: Correct way to get current datetime
            "created_at": datetime.now().isoformat(timespec='microseconds'),
            "material_line": [
                {
                    "name": conf.get('name', 'Unknown'),
                    "material_type": conf.get('material_type', 'Unknown'),
                    "ref_layer": conf.get('ref_layer', 'None')
                } for conf in self.material_configs
            ]
        }

        try:
            with open(config_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=4, ensure_ascii=False)
            self.message_text.append("Updated material_lines_config.txt in construction layer root")
        except Exception as e:
            self.message_text.append(f"Failed to update material_lines_config.txt: {str(e)}")

# =====================================================================================================================================
    def get_real_coordinates_from_chainage(self, chainage_m):
        """
        Return real-world (X, Y, Z) coordinates for a given chainage in meters.
        This is the PRIMARY method used everywhere for accurate positioning.
        """
        if not hasattr(self, 'horizontal_alignment') or not self.horizontal_alignment:
            return None, None, None
        if not hasattr(self, 'vertical_profile') or not self.vertical_profile:
            return None, None, None

        try:
            X, Y = self.horizontal_alignment.get_xy(chainage_m)  # Accurate horizontal position
        except:
            X, Y = None, None

        try:
            Z = self.vertical_profile.get_elevation(chainage_m)  # Accurate vertical elevation
        except:
            Z = None

        return X, Y, Z

# =====================================================================================================================================
    def get_km_and_interval(self, chainage_m):
        """Return km (int) and interval meters (float) from total chainage"""
        if not hasattr(self, 'zero_start_km') or self.zero_start_km is None:
            return 0, chainage_m

        km = int(self.zero_start_km)
        interval = chainage_m - (km * 1000)
        return km, round(interval, 3)

# ============================================================================================================================
    # New/Updated unified handler (add to class)
    def on_graph_click(self, event):
        """Unified mouse click handler for all drawing modes"""
        if event.inaxes != self.ax:
            return

        x, y = event.xdata, event.ydata
        if x is None or y is None:
            return

        if not self.zero_line_set:
            self.message_text.append("Set zero line first before drawing points.")
            return

        if not (0 <= x <= self.total_distance):
            self.message_text.append(f"Click outside chainage range (0-{self.total_distance:.1f}m).")
            return

        if self.active_line_type == 'construction':
            self.on_construction_click(event)
        elif self.active_line_type == 'material' and self.active_material_index is not None:
            self.on_material_draw_click(event) 
        else:
            self.message_text.append("No active drawing mode selected.")

# =======================================================================================================================================
    # Construction click handler
    def on_construction_click(self, event):
        """Handle click for new construction dots (existing logic - provided for reference)"""
        # This mirrors your existing construction plotting (adapt if your exact code differs)
        if not self.construction_mode_active:  # Or whatever guard you have
            return
        x, y = event.xdata, event.ydata
        if x is None or y is None:
            return
        next_num = getattr(self, 'construction_point_number', 1)  # Assuming you track this

        # Create dot
        dot_artist = self.ax.plot(x, y, 'o', color='red', markersize=6, picker=True)[0]
        dot_artist.set_pickradius(10)
        dot_artist.point_data = {
            'type': 'construction_dot',
            'point_number': next_num,
            'x': x,
            'y': y,
            'config': {}
        }

        # Create label
        label_text = f"P{next_num}"
        label_y = y + 0.15
        label_artist = self.ax.annotate(label_text, (x, label_y),
                                        xytext=(0, 8), textcoords='offset points',
                                        bbox=dict(boxstyle='round,pad=0.3', facecolor='red', alpha=0.7),
                                        ha='center', fontsize=8, picker=True)
        # Note: No set_pickradius for annotation
        label_artist.point_data = dot_artist.point_data

        # Store (assuming you have self.construction_dots = [] in init)
        if not hasattr(self, 'construction_dots'):
            self.construction_dots = []
        self.construction_dots.append({
            'x': x, 'y': y, 'config': {}, 'label': label_artist,
            'point_number': next_num
        })

        # Update number
        self.construction_point_number = next_num + 1

        # Store in line_types
        self.line_types['construction']['artists'].append(dot_artist)
        self.line_types['construction']['polylines'].append([(x, y)])

        self.canvas.draw_idle()
        self.message_text.append(f"Construction dot P{next_num} placed at {x:.2f}m. Click to configure.")

# =======================================================================================================================================================
    # UPDATE toggle for construction (if exists, add similar activation)
    def toggle_construction_line(self, state):  # Assuming this exists for construction checkbox
        """Toggle construction mode like material"""
        if state == Qt.Checked:
            self.active_line_type = 'construction'
            self.construction_mode_active = True
            self.message_text.append("Construction mode activated for drawing.")
            if not hasattr(self, 'construction_point_number'):
                self.construction_point_number = 1
        else:
            self.active_line_type = None
            self.construction_mode_active = False
            self.message_text.append("Construction mode deactivated.")

# ============================================================================================================================================================
    # Define function for the load baseline from design layer:    
    def _load_baseline_from_design_layer(self, material_config):
        """
        Load the reference baseline JSON for a material line.
        Uses the display name stored in material_config['ref_layer']
        (e.g. "construction", "road_surface", etc.).
        Now safely handles ref_layer being a list (from NewMaterialLineDialog).
        Matching is case-insensitive to fix 'Construction' vs 'construction'.
        """
        ref_layer = material_config.get('ref_layer', 'construction')
        
        # Handle list safely
        if isinstance(ref_layer, list):
            if not ref_layer:
                ref_layer = 'construction'
            else:
                ref_layer = ref_layer[0]  # Use first selected reference for elevation
        
        selected_display_name = str(ref_layer).strip()
        
        if not selected_display_name or selected_display_name.lower() in ("", "none"):
            self.message_text.append("No reference baseline selected — using fallback.")
            return None, None

        if not hasattr(self, 'current_construction_layer_path') or not self.current_construction_layer_path:
            self.message_text.append("Current construction layer path not set.")
            return None, None

        config_path = os.path.join(self.current_construction_layer_path, "Construction_Layer_config.txt")
        if not os.path.exists(config_path):
            self.message_text.append(f"Construction_Layer_config.txt not found: {config_path}")
            return None, None

        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config = json.load(f)
        except Exception as e:
            self.message_text.append(f"Failed to read Construction_Layer_config: {str(e)}")
            return None, None

        reference_layer_2d = config.get("reference_layer_2d")
        if not reference_layer_2d:
            self.message_text.append("reference_layer_2d not defined in config.")
            return None, None

        selected_baselines = config.get("base_lines_reference", [])
        if not selected_baselines:
            self.message_text.append("No baselines listed in base_lines_reference.")
            return None, None

        # Build absolute path to the design layer
        worksheet_root = os.path.abspath(os.path.join(self.current_construction_layer_path, "..", ".."))
        designs_folder = os.path.join(worksheet_root, "designs")
        design_layer_path = os.path.join(designs_folder, reference_layer_2d)

        if not os.path.exists(design_layer_path):
            self.message_text.append(f"Design layer folder not found: {design_layer_path}")
            return None, None

        # Find the filename that matches the display name (case-insensitive)
        target_filename = None
        for baseline_file in selected_baselines:
            display_name = os.path.basename(baseline_file).replace("_baseline.json", "")
            if display_name.lower() == selected_display_name.lower():
                target_filename = baseline_file
                break

        if not target_filename:
            self.message_text.append(f"Baseline '{selected_display_name}' not found in selected baselines. Using fallback.")
            return None, None

        baseline_path = os.path.join(design_layer_path, target_filename)
        if not os.path.exists(baseline_path):
            self.message_text.append(f"Baseline file missing: {baseline_path}")
            return None, None

        try:
            with open(baseline_path, 'r', encoding='utf-8') as f:
                data = json.load(f)

            # Extract points – supports both new and old baseline formats
            all_points = []
            if "polylines" in data and data["polylines"]:
                for poly in data["polylines"]:
                    if "points" in poly:
                        all_points.extend(poly["points"])
            elif "points" in data:
                all_points = data["points"]

            if len(all_points) < 2:
                self.message_text.append(f"Baseline has insufficient points ({len(all_points)}).")
                return None, None

            xs = [pt.get("chainage_m", pt[0] if isinstance(pt, list) else 0) for pt in all_points]
            ys = [pt.get("relative_elevation_m", pt[1] if isinstance(pt, list) else 0) for pt in all_points]

            self.message_text.append(
                f"Baseline loaded: '{selected_display_name}' ({len(xs)} points)"
            )
            return xs, ys

        except Exception as e:
            self.message_text.append(f"Error reading baseline JSON: {str(e)}")
            return None, None

# ==============================================================================================================================================
# Method to load and draw saved material filling, labels, and 3D
    def load_and_draw_material_filling(self, material_index):
        """Called when a material line is activated – redraws saved filling from latest JSON."""
        import json
        import os

        folder_name = self.material_configs[material_index].get('folder_name')
        if not folder_name:
            self.message_text.append("Material folder name missing.")
            return

        json_path = os.path.join(self.current_construction_layer_path, f"{folder_name}.json")
        if not os.path.exists(json_path):
            # Try with normalized name if not found
            normalized_name = folder_name.lower().replace(' ', '_')
            json_path_normalized = os.path.join(self.current_construction_layer_path, f"{normalized_name}.json")
            if os.path.exists(json_path_normalized):
                json_path = json_path_normalized
                self.message_text.append(f"Using normalized JSON path: {json_path}")
            else:
                self.message_text.append(f"Material JSON not found: {json_path}")
                return

        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            self.message_text.append(f"Error loading saved material JSON: {str(e)}")
            return

        # Clear existing polyline artists
        if material_index in self.material_polylines_artists:
            for artist in self.material_polylines_artists[material_index]:
                try:
                    artist.remove()
                except:
                    pass
            self.material_polylines_artists[material_index] = []

        # Draw permanent polylines from segments
        for seg in data.get("segments", []):
            pts = seg.get("polyline_points", [])
            if len(pts) < 2:
                continue
            xs = [p["chainage_m"] for p in pts]
            ys = [p["relative_elevation_m"] for p in pts]
            permanent_line = self.ax.plot(
                xs, ys,
                color='orange',
                linewidth=3,
                linestyle='-',
                marker='o',
                markersize=6,
                markerfacecolor='orange',
                markeredgecolor='darkred',
                alpha=0.9
            )[0]
            self.material_polylines_artists.setdefault(material_index, []).append(permanent_line)

        from_m = data["overall_from_chainage"]["chainage_m"]
        to_m   = data["overall_to_chainage"]["chainage_m"]

        # Use values from first segment (or 0 if none)
        width_m = 0.0
        if "segments" in data and data["segments"]:
            width_m = data["segments"][0].get("width_m", 0.0)

        # Redraw everything
        self.draw_material_filling(
            from_chainage_m=from_m,
            to_chainage_m=to_m,
            thickness_m=0.0,  # not used for height anymore
            material_index=material_index,
            material_config=self.material_configs[material_index],
            color='#FF9800',
            alpha=0.6,
            width_m=width_m
        )

        # Re-create segment labels
        if material_index not in self.material_segment_labels:
            self.material_segment_labels[material_index] = []

        for label in self.material_segment_labels.get(material_index, []):
            label.remove()
        self.material_segment_labels[material_index] = []

        if "segments" in data and data["segments"]:
            for seg in data["segments"]:
                seg_num = seg["segment_number"]
                from_seg = seg["from_chainage_m"]
                to_seg   = seg["to_chainage_m"]
                mid_x = (from_seg + to_seg) / 2

                # Approximate label Y from polyline points
                pts = seg.get("polyline_points", [])
                ys = [p["relative_elevation_m"] for p in pts]
                mid_y = (max(ys) + 0.25) if ys else 0.25

                label_text = f"M{material_index+1}-{seg_num}"
                if seg.get("material_thickness_m", 0) > 0:
                    label_text += f" ({seg['material_thickness_m']*1000:.0f}mm)"

                annot = self.ax.annotate(
                    label_text, (mid_x, mid_y),
                    xytext=(0, 12), textcoords='offset points',
                    ha='center', va='bottom',
                    fontsize=9, fontweight='bold', color='white',
                    bbox=dict(boxstyle='round,pad=0.4',
                            facecolor='lightgreen',
                            alpha=0.9, edgecolor='green'),
                    picker=10
                )
                annot.point_data = {
                    'type': 'material_segment_label',
                    'material_index': material_index,
                    'segment_number': seg_num,
                    'from_chainage_m': from_seg,
                    'to_chainage_m': to_seg,
                    'config': seg
                }
                self.material_segment_labels[material_index].append(annot)

        self.canvas.draw_idle()

# =======================================================================================================================================
# NEW: Method to create 3D virtual top surface for material
    def create_3d_material_surface(self, material_index, xs, ys, width):
        """Create a 3D polygon surface (virtual plane) for the material top with given width, centered on the alignment."""
        import vtk
        import numpy as np

        if not hasattr(self, 'start_point') or not hasattr(self, 'end_point') or not hasattr(self, 'total_distance'):
            self.message_text.append("Cannot create 3D surface: zero line not set.")
            return

        start = np.array(self.start_point)
        end = np.array(self.end_point)
        dir_vec = (end - start) / self.total_distance
        dir_xy_norm = dir_vec[0:2] / np.linalg.norm(dir_vec[0:2])
        perp_xy = np.array([-dir_xy_norm[1], dir_xy_norm[0]])
        perp = np.array([perp_xy[0], perp_xy[1], 0.0])

        num_points = len(xs)
        left_points = []
        right_points = []

        for i in range(num_points):
            chainage = xs[i]
            t = chainage / self.total_distance
            center_base = start + t * (end - start)
            abs_z = center_base[2] + ys[i]  # Absolute Z = zero line Z + relative elevation
            center = np.array([center_base[0], center_base[1], abs_z])
            left = center + perp * (width / 2)
            right = center - perp * (width / 2)
            left_points.append(left)
            right_points.append(right)

        # Create VTK points
        vtk_points = vtk.vtkPoints()
        point_id = 0
        left_ids = []
        right_ids = []
        for pt in left_points:
            vtk_points.InsertNextPoint(pt)
            left_ids.append(point_id)
            point_id += 1
        for pt in right_points:
            vtk_points.InsertNextPoint(pt)
            right_ids.append(point_id)
            point_id += 1

        # Create quad strips
        polys = vtk.vtkCellArray()
        for i in range(num_points - 1):
            quad = vtk.vtkQuad()
            quad.GetPointIds().SetId(0, left_ids[i])
            quad.GetPointIds().SetId(1, left_ids[i+1])
            quad.GetPointIds().SetId(2, right_ids[i+1])
            quad.GetPointIds().SetId(3, right_ids[i])
            polys.InsertNextCell(quad)

        # PolyData
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(vtk_points)
        polydata.SetPolys(polys)

        # Mapper and Actor
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(polydata)

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        color = self.plane_colors.get('material', (1.0, 1.0, 0.0, 0.4))
        actor.GetProperty().SetColor(color[0], color[1], color[2])
        actor.GetProperty().SetOpacity(color[3])

        # Add to renderer
        self.renderer.AddActor(actor)
        self.vtkWidget.GetRenderWindow().Render()

        # Store for cleanup
        if material_index not in self.material_3d_actors:
            self.material_3d_actors[material_index] = []
        self.material_3d_actors[material_index].append(actor)

# ==============================================================================================================================================
    # NEW HELPER: Load a design baseline by display name (e.g., "Construction")
    def _load_design_baseline(self, design_name):
        import json
        import os
        if not hasattr(self, 'current_construction_layer_path') or not self.current_construction_layer_path:
            self.message_text.append("Current construction layer path not set.")
            return [], []
        config_path = os.path.join(self.current_construction_layer_path, "Construction_Layer_config.txt")
        if not os.path.exists(config_path):
            self.message_text.append(f"Construction_Layer_config.txt not found: {config_path}")
            return [], []
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config = json.load(f)
        except Exception as e:
            self.message_text.append(f"Failed to read Construction_Layer_config: {str(e)}")
            return [], []
        reference_layer_2d = config.get("reference_layer_2d")
        if not reference_layer_2d:
            self.message_text.append("reference_layer_2d not defined in config.")
            return [], []
        selected_baselines = config.get("base_lines_reference", [])
        if not selected_baselines:
            self.message_text.append("No baselines listed in base_lines_reference.")
            return [], []
        # Build absolute path to the design layer
        worksheet_root = os.path.abspath(os.path.join(self.current_construction_layer_path, "..", ".."))
        designs_folder = os.path.join(worksheet_root, "designs")
        design_layer_path = os.path.join(designs_folder, reference_layer_2d)
        if not os.path.exists(design_layer_path):
            self.message_text.append(f"Design layer folder not found: {design_layer_path}")
            return [], []
        # Find the filename that matches the display name (case-insensitive)
        target_filename = None
        for baseline_file in selected_baselines:
            display_name = os.path.basename(baseline_file).replace("_baseline.json", "")
            if display_name.lower() == design_name.lower():
                target_filename = baseline_file
                break
        if not target_filename:
            self.message_text.append(f"Baseline '{design_name}' not found in selected baselines.")
            return [], []
        baseline_path = os.path.join(design_layer_path, target_filename)
        if not os.path.exists(baseline_path):
            self.message_text.append(f"Baseline file missing: {baseline_path}")
            return [], []
        try:
            with open(baseline_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            # Extract points – supports both new and old baseline formats
            all_points = []
            if "polylines" in data and data["polylines"]:
                for poly in data["polylines"]:
                    if "points" in poly:
                        all_points.extend(poly["points"])
            elif "points" in data:
                all_points = data["points"]
            if len(all_points) < 2:
                self.message_text.append(f"Baseline has insufficient points ({len(all_points)}).")
                return [], []
            xs = [pt.get("chainage_m", pt[0] if isinstance(pt, list) else 0) for pt in all_points]
            ys = [pt.get("relative_elevation_m", pt[1] if isinstance(pt, list) else 0) for pt in all_points]
            self.message_text.append(
                f"Design baseline loaded: '{design_name}' ({len(xs)} points)"
            )
            return xs, ys
        except Exception as e:
            self.message_text.append(f"Error reading baseline JSON: {str(e)}")
            return [], []

# ==============================================================================================================================================
    # NEW HELPER: Load a previous material's top as baseline (from its JSON)
    def _load_material_top_as_baseline(self, mat_name):
        import json
        import os
        import numpy as np
        mat_filename = mat_name.lower().strip() + ".json"
        json_path = os.path.join(self.current_construction_layer_path, mat_filename)
        if not os.path.exists(json_path):
            # Try removing 'm' prefix if present
            if mat_name.lower().startswith("m"):
                mat_filename = mat_name.lower()[1:] + ".json"  # e.g., "1.json"
                json_path = os.path.join(self.current_construction_layer_path, mat_filename)
            if not os.path.exists(json_path):
                self.message_text.append(f"Material JSON for {mat_name} not found.")
                return [], []
        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            self.message_text.append(f"Error loading material JSON for {mat_name}: {str(e)}")
            return [], []
        all_xs = []
        all_ys = []
        for seg in data.get("segments", []):
            for p in seg.get("polyline_points", []):
                all_xs.append(p["chainage_m"])
                all_ys.append(p["relative_elevation_m"])
        if not all_xs:
            self.message_text.append(f"No points found in material JSON for {mat_name}.")
            return [], []
        # Sort
        sorted_idx = np.argsort(all_xs)
        xs = np.array(all_xs)[sorted_idx]
        ys = np.array(all_ys)[sorted_idx]
        self.message_text.append(
            f"Material top loaded as baseline: '{mat_name}' ({len(xs)} points)"
        )
        return xs.tolist(), ys.tolist()

# ==============================================================================================================================================
    # Define function for the draw material filling:    
    def draw_material_filling(self, from_chainage_m, to_chainage_m, thickness_m,
                            material_index, material_config, color='#FF9800', alpha=0.6, width_m=0.0,
                            hatch_pattern=None, base_elevation=0.0):
        """
        Draw 2D hatching + 3D volume for the material.
        Now computes effective bottom as max over all referenced previous layers' tops.
        Auto-assigns different hatching per material_index.
        All original functionality preserved.
        """
        import numpy as np
        from matplotlib.patches import Polygon
        import vtk
        # === Auto-assign unique hatching per material ===
        HATCH_PATTERNS = ['o', 'x', '/', '+', '-', '.', '*', '\\', 'O', '|']
        if hatch_pattern is None:
            hatch_pattern = HATCH_PATTERNS[material_index % len(HATCH_PATTERNS)]
        # Initialise containers if needed
        if not hasattr(self, 'material_fill_patches'):
            self.material_fill_patches = {}
        if not hasattr(self, 'material_3d_actors'):
            self.material_3d_actors = {}
        # Clean previous drawings for this material
        if material_index in self.material_fill_patches:
            for p in self.material_fill_patches[material_index]:
                for patch in p.values():
                    if patch in self.ax.patches:
                        patch.remove()
            self.material_fill_patches[material_index] = []
        if material_index in self.material_3d_actors:
            for actor in self.material_3d_actors[material_index]:
                self.renderer.RemoveActor(actor)
            self.material_3d_actors[material_index] = []
        # Load material top line (drawn by user)
        mat_xs, mat_ys = self._get_material_line_points_for_segment(material_index, from_chainage_m, to_chainage_m)
        if len(mat_xs) < 2:
            self.message_text.append("Not enough material points to draw filling.")
            return
        x_dense = np.array(mat_xs)
        top_y_dense = np.array(mat_ys)
        # === Compute effective bottom from all references ===
        ref_layer = material_config.get('ref_layer')
        if not isinstance(ref_layer, list):
            ref_layer = [ref_layer] if ref_layer else []
        all_interp_ys = []
        for ref in ref_layer:
            if str(ref).lower() == "construction":
                ref_xs, ref_ys = self._load_design_baseline("Construction")
            else:
                ref_xs, ref_ys = self._load_material_top_as_baseline(ref)
            if ref_xs and len(ref_xs) >= 2:
                # Sort just in case
                sorted_idx = np.argsort(ref_xs)
                ref_xs = np.array(ref_xs)[sorted_idx]
                ref_ys = np.array(ref_ys)[sorted_idx]
                # Interp with nan outside range
                interp_y = np.full_like(x_dense, np.nan)
                mask = (x_dense >= ref_xs[0]) & (x_dense <= ref_xs[-1])
                interp_y[mask] = np.interp(x_dense[mask], ref_xs, ref_ys)
                all_interp_ys.append(interp_y)
        if all_interp_ys:
            # Find the highest top among all referenced layers. 
            # Suppress "All-NaN axis" warning if some points have no reference data; 
            # nan_to_num handles those points correctly on the next line.
            with np.errstate(all='ignore'):
                bottom_y_dense = np.nanmax(all_interp_ys, axis=0)
            bottom_y_dense = np.nan_to_num(bottom_y_dense, nan=base_elevation)
        else:
            bottom_y_dense = np.full_like(x_dense, base_elevation)
        top_y_dense = np.maximum(top_y_dense, bottom_y_dense)  # ensure top >= bottom
        # ---------- 2D Hatching ----------
        vertices = np.vstack([
            np.column_stack([x_dense, bottom_y_dense]),
            np.column_stack([x_dense[::-1], top_y_dense[::-1]])
        ])
        bg = Polygon(vertices, closed=True, facecolor=color, alpha=0.35, edgecolor='none', zorder=2)
        hatch = Polygon(vertices, closed=True, facecolor='none', hatch=hatch_pattern, edgecolor='black', linewidth=0.8, zorder=3)
        self.ax.add_patch(bg)
        self.ax.add_patch(hatch)
        self.material_fill_patches.setdefault(material_index, []).append({'bg': bg, 'hatch': hatch})
        self.canvas.draw_idle()
        avg_thick_mm = np.mean(top_y_dense - bottom_y_dense) * 1000
        self.message_text.append(f"2D filling drawn: avg thickness {avg_thick_mm:.0f} mm")
        # ---------- 3D Volume (only if width > 0) ----------
        if width_m <= 0.01:
            self.message_text.append("Width ≤ 0 → no 3D volume created.")
            return
        # Build 3D points with proper perpendicular offset
        delta = 0.5
        half_width = width_m / 2.0
        left_top_pts, right_top_pts = [], []
        left_bottom_pts, right_bottom_pts = [], []
        for i in range(len(x_dense)):
            ch = x_dense[i]
            X, Y, base_Z = self.get_real_coordinates_from_chainage(ch)
            if X is None:
                X, Y, base_Z = self.interpolate_xyz(ch)
            # Tangent approximation
            X1, Y1, _ = self.get_real_coordinates_from_chainage(max(0, ch - delta))
            X2, Y2, _ = self.get_real_coordinates_from_chainage(min(self.total_distance, ch + delta))
            if X1 is None: X1, Y1, _ = self.interpolate_xyz(max(0, ch - delta))
            if X2 is None: X2, Y2, _ = self.interpolate_xyz(min(self.total_distance, ch + delta))
            dir_vec = np.array([X2 - X1, Y2 - Y1])
            len_dir = np.linalg.norm(dir_vec)
            if len_dir < 1e-6:
                continue
            dir_norm = dir_vec / len_dir
            perp_norm_2d = np.array([-dir_norm[1], dir_norm[0]])
            offset = np.array([perp_norm_2d[0], perp_norm_2d[1], 0.0]) * half_width
            top_z = base_Z + top_y_dense[i]
            bottom_z = base_Z + bottom_y_dense[i]
            center_top = np.array([X, Y, top_z])
            center_bottom = np.array([X, Y, bottom_z])
            left_top_pts.append(center_top + offset)
            right_top_pts.append(center_top - offset)
            left_bottom_pts.append(center_bottom + offset)
            right_bottom_pts.append(center_bottom - offset)
        if len(left_top_pts) < 2:
            self.message_text.append("Not enough valid 3D points for volume.")
            return
        # VTK mesh construction (unchanged – proven stable)
        points = vtk.vtkPoints()
        cells = vtk.vtkCellArray()
        lt_ids = [points.InsertNextPoint(p) for p in left_top_pts]
        rt_ids = [points.InsertNextPoint(p) for p in right_top_pts]
        lb_ids = [points.InsertNextPoint(p) for p in left_bottom_pts]
        rb_ids = [points.InsertNextPoint(p) for p in right_bottom_pts]
        n = len(lt_ids)
        def add_quad(a, b, c, d):
            quad = vtk.vtkQuad()
            quad.GetPointIds().SetId(0, a)
            quad.GetPointIds().SetId(1, b)
            quad.GetPointIds().SetId(2, c)
            quad.GetPointIds().SetId(3, d)
            cells.InsertNextCell(quad)
        for i in range(n-1):
            add_quad(lt_ids[i], lt_ids[i+1], rt_ids[i+1], rt_ids[i]) # top
            add_quad(lb_ids[i], lb_ids[i+1], rb_ids[i+1], rb_ids[i]) # bottom
            add_quad(lt_ids[i], lt_ids[i+1], lb_ids[i+1], lb_ids[i]) # left wall
            add_quad(rt_ids[i], rt_ids[i+1], rb_ids[i+1], rb_ids[i]) # right wall
        # End caps
        add_quad(lt_ids[0], rt_ids[0], rb_ids[0], lb_ids[0])
        add_quad(lt_ids[-1], lb_ids[-1], rb_ids[-1], rt_ids[-1])
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(points)
        polydata.SetPolys(cells)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(polydata)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(1.0, 0.65, 0.0) # orange
        actor.GetProperty().SetOpacity(0.5)
        self.renderer.AddActor(actor)
        # Safe render
        if hasattr(self, 'vtkWidget') and self.vtkWidget:
            rw = self.vtkWidget.GetRenderWindow()
            if rw: rw.Render()
        self.material_3d_actors.setdefault(material_index, []).append(actor)
        self.message_text.append(f"3D material volume created (width {width_m:.1f} m)")

# ====================================================================================================================================================================
    # Define function for the load baseline and recreate curves:    
    def load_baseline_and_recreate_curves(self, json_path, ltype):
        """
        Loads a single baseline JSON file (e.g., surface_baseline.json).
        - Restores polylines into self.line_types
        - Restores width
        - Recreates self.curve_labels from points that have angle_deg
        - Recreates yellow curve labels on 2D graph
        This ensures that when user opens old worksheet, curved road appears correctly in 3D.
        """
        if not os.path.exists(json_path):
            return False

        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            self.message_text.append(f"Error reading {os.path.basename(json_path)}: {str(e)}")
            return False

        # === Restore width ===
        width = data.get("width_meters", 0.0)
        if width > 0:
            self.baseline_widths[ltype] = width

        # === Restore polylines (for 2D drawing and 3D mapping) ===
        polylines_2d = []
        for poly in data.get("polylines", []):
            poly_2d = []
            for pt in poly.get("points", []):
                chainage = pt.get("chainage_m", 0.0)
                rel_elev = pt.get("relative_elevation_m", 0.0)
                poly_2d.append((chainage, rel_elev))
            if len(poly_2d) >= 2:
                polylines_2d.append(poly_2d)
        self.line_types[ltype]['polylines'] = polylines_2d

        # === RECREATE CURVE LABELS FROM POINT-EMBEDDED ANGLES ===
        #self.clear_curve_labels()  # Clear any previous
        recreated_count = 0
        for poly in data.get("polylines", []):
            for pt in poly.get("points", []):
                if "angle_deg" in pt:
                    chainage = pt["chainage_m"]
                    config = {
                        'angle': pt["angle_deg"],
                        'inner_curve': pt.get("inner_curve", False),
                        'outer_curve': pt.get("outer_curve", False)
                    }
                    # Add to curve_labels list
                    self.curve_labels.append({
                        'chainage': chainage,
                        'config': config
                    })
                    # Recreate yellow label on 2D graph
                    self.add_curve_label_at_x(chainage, config)
                    recreated_count += 1

        if recreated_count > 0:
            self.message_text.append(f"Recreated {recreated_count} curve label(s) from saved angles in {os.path.basename(json_path)}")
        else:
            self.message_text.append(f"Loaded straight baseline: {os.path.basename(json_path)}")

        # Optional: redraw on graph
        self.redraw_baseline_on_graph(ltype)

        return True
# # ====================================================================================================================================================================
# #                                                                       *** END OF APPLICAATION ***
# # ====================================================================================================================================================================      