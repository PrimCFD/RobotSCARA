from robot.gui import create_Main_Window


def main():
    """
    Main application entry point for SCARA Robot GUI.

    Exports the main window creation function and handles application initialization.
    """
    resolution = (1400, 900)
    fps = 30
    create_Main_Window(resolution, fps)
