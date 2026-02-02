# main.py
import sys
import os

from PyQt5.QtWidgets import QApplication, QDialog

from pointcloudviewer import PointCloudViewer
from login import LoginDialog
from welcome_page import WelcomePage


def main():
    app = QApplication(sys.argv)

    # Show login dialog first
    login_dlg = LoginDialog()
    if login_dlg.exec_() != LoginDialog.Accepted:
        # User cancelled login
        sys.exit(0)

    username = login_dlg.logged_in_username  # This attribute was already set in login.py
    user_id = login_dlg.logged_in_user_id    # Get user_id for worksheet folder creation
    
    # Get full name from user config file
    user_full_name = username  # Default to username
    try:
        import json
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
        sys.exit(0)

    # Show main application window maximized (with taskbar visible)
    window = PointCloudViewer(username=username, user_id=user_id, user_full_name=user_full_name)
    window.showMaximized()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()