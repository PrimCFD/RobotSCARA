from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QTreeView, QFileSystemModel, QPushButton, QInputDialog, \
    QMessageBox, QFileDialog, QLineEdit, QMenu, QAbstractItemView, QStyle, QShortcut
from PyQt5.QtGui import QIcon, QKeySequence
from PyQt5.QtCore import QDir, Qt, pyqtSignal
import os
import shutil
import time
import csv, json
from pathlib import Path
import pandas as pd
import numpy as np
from robot.misc import constants, load_config
import hashlib


class TrajectoryLogger:
    def __init__(self):
        package_root = Path(__file__).parent.parent
        self.buffer_path = package_root / "buffer" / "buffer_measure.csv"
        self.buffer_path.parent.mkdir(parents=True, exist_ok=True)

        self.current_data = []
        self.start_time = None
        self.results = None
        self.sim_results = None
        self.sim_hash = None

        self.empty_buffer_measure = True
        self.empty_file_measure = True

        # Create the file if it doesn't exist
        if not self.buffer_path.exists():
            with open(self.buffer_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["t", "theta", "phi"])

        self.read_buffer()

    def start_new_trajectory(self):
        self.current_data = []
        self.start_time = time.time()

    def log_frame(self, angles):
        self.empty = False
        if self.start_time is None:
            raise RuntimeError("Call start_new_trajectory() before logging frames.")
        timestamp = time.time() - self.start_time
        self.current_data.append([timestamp] + list(angles))

    def save_to_buffer(self):
        with open(self.buffer_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["t", "theta", "phi"])
            writer.writerows(self.current_data)
        self.read_buffer()

    def read_buffer(self):
        try:
            if not self.buffer_path.exists():
                self.empty_buffer_measure = True
                return

            df = pd.read_csv(self.buffer_path)
            t = df['t'].to_numpy()
            theta_arc = df['theta'].to_numpy()
            phi_arc = df['phi'].to_numpy()

            self.empty_buffer_measure = not (len(t) > 0 and len(phi_arc) > 0 and len(theta_arc) > 0)
            self.results = (constants(), t, phi_arc, theta_arc)

        except Exception as e:
            self.empty_buffer_measure = True
            QMessageBox.critical(None, "CSV Error", f"Failed to load trajectory CSV:\n{e}")

    def delete_buffer(self):
        self.empty = True
        self.current_data = []
        self.save_to_buffer()


class SimulationLogger:
    def __init__(self):
        package_root = Path(__file__).parent.parent
        buffer_dir = package_root / "buffer"
        buffer_dir.mkdir(parents=True, exist_ok=True)

        self.simulation_buffer_path = buffer_dir / "buffer_simulation.csv"
        self.sim_hash_path = buffer_dir / "buffer_hash.txt"

        self.sim_results = None
        self.sim_hash = None
        self.empty_buffer_simulation = True

        # Ensure simulation files exist
        if not self.simulation_buffer_path.exists():
            with open(self.simulation_buffer_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "t", "x0", "x1", "x2",
                    "x0_dot", "x1_dot", "x2_dot",
                    "theta0", "theta1", "theta2",
                    "theta0_dot", "theta1_dot", "theta2_dot",
                    "tau0", "tau1", "tau2"
                ])
        if not self.sim_hash_path.exists():
            with open(self.sim_hash_path, 'w') as f:
                f.write("")

        self.load_cached_simulation()

    # In measurement.py > SimulationLogger class
    def compute_trajectory_hash(self, trajectory_data):
        # Include robot configuration in the hash
        config = load_config()  # Get current robot parameters
        serialized = json.dumps({
            "trajectory": trajectory_data,
            "config": [config]  # Add config parameters
        }, sort_keys=True)
        return hashlib.sha256(serialized.encode()).hexdigest()

    def save_simulation_results(self, results, hash_value):
        self.sim_results = results
        self.sim_hash = hash_value

        with open(self.simulation_buffer_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=[
                "t", "x0", "x1", "x2", "x0_dot", "x1_dot", "x2_dot", "theta0", "theta1", "theta2", "theta0_dot", "theta1_dot", "theta2_dot", "tau0", "tau1", "tau2"
            ])
            writer.writeheader()
            for frame in results:
                writer.writerow({
                    "t": frame["t"],
                    "x0": frame["x"][0], "x1": frame["x"][1], "x2": frame["x"][2],
                    "x0_dot": frame["x_dot"][0], "x1_dot": frame["x_dot"][1], "x2_dot": frame["x_dot"][2],
                    "theta0": frame["theta"][0], "theta1": frame["theta"][1], "theta2": frame["theta"][2],
                    "theta0_dot": frame["theta_dot"][0], "theta1_dot": frame["theta_dot"][1], "theta2_dot": frame["theta_dot"][2],
                    "tau0": frame["tau"][0], "tau1": frame["tau"][1], "tau2": frame["tau"][2],
                })

        with open(self.sim_hash_path, 'w') as f:
            f.write(self.sim_hash)

    def load_cached_simulation(self):
        try:
            if not self.simulation_buffer_path.exists() or not self.sim_hash_path.exists():
                self.empty_buffer_simulation = True
                return False

            with open(self.sim_hash_path) as f:
                self.sim_hash = f.read().strip()

            with open(self.simulation_buffer_path) as f:
                reader = list(csv.DictReader(f))
                if not reader:
                    self.empty_buffer_simulation = True
                    return False

                self.sim_results = []
                for row in reader:
                    try:
                        frame = {
                            "t": float(row["t"]),
                            "x": [float(row["x0"]), float(row["x1"]), float(row["x2"])],
                            "x_dot": [float(row["x0_dot"]), float(row["x1_dot"]), float(row["x2_dot"])],
                            "theta": [float(row["theta0"]), float(row["theta1"]), float(row["theta2"])],
                            "theta_dot": [float(row["theta0_dot"]), float(row["theta1_dot"]), float(row["theta2_dot"])],
                            "tau": [float(row["tau0"]), float(row["tau1"]), float(row["tau2"])],
                        }
                        self.sim_results.append(frame)
                    except (KeyError, ValueError) as e:
                        print(f"[Cache] Malformed row in simulation buffer: {e}")
                        self.empty_buffer_simulation = True
                        return False

                self.empty_buffer_simulation = len(self.sim_results) == 0
                return not self.empty_buffer_simulation

        except Exception as e:
            print(f"[Cache] Failed to load simulation cache: {e}")
            self.empty_buffer_simulation = True
            return False

    def needs_recompute(self, new_hash):
        return new_hash != self.sim_hash




class FileBrowser(QWidget):
    traj_constants_ready = pyqtSignal(tuple)

    def __init__(self, trajectory_logger):
        super().__init__()
        self.trajectory_logger = trajectory_logger

        self.layout = QVBoxLayout(self)

        self.model = QFileSystemModel()
        self.model.setRootPath(QDir.rootPath())

        self.tree = QTreeView()
        self.tree.setModel(self.model)

        package_root = Path(__file__).parent.parent
        self.data_root = package_root / "data"
        os.makedirs(str(self.data_root), exist_ok=True)  # Make sure it exists
        self.tree.setRootIndex(self.model.index(str(self.data_root)))
        self.tree.doubleClicked.connect(self.file_selected)
        self.tree.setSelectionMode(self.tree.SingleSelection)

        self.tree.setContextMenuPolicy(Qt.CustomContextMenu)
        self.tree.customContextMenuRequested.connect(self.show_context_menu)

        # --- Breadcrumb Bar ---
        self.breadcrumb = QHBoxLayout()
        self.breadcrumb_widget = QWidget()
        self.breadcrumb_widget.setLayout(self.breadcrumb)

        # Button rows
        button_layout_top = QHBoxLayout()
        self.new_exp_btn = QPushButton("New Experiment")

        button_layout = QHBoxLayout()
        self.rename_btn = QPushButton("Rename")
        self.delete_btn = QPushButton("Delete")
        self.save_btn = QPushButton("Save")
        self.play_btn = QPushButton("Play trajectory")

        # Set icons using standard icons from QStyle
        self.new_exp_btn.setIcon(self.style().standardIcon(QStyle.SP_FileDialogNewFolder))
        self.rename_btn.setIcon(self.style().standardIcon(QStyle.SP_FileIcon))
        self.delete_btn.setIcon(self.style().standardIcon(QStyle.SP_TrashIcon))
        self.save_btn.setIcon(self.style().standardIcon(QStyle.SP_DialogSaveButton))

        button_layout_top.addWidget(self.new_exp_btn)

        button_layout.addWidget(self.rename_btn)
        button_layout.addWidget(self.delete_btn)
        button_layout.addWidget(self.save_btn)

        # --- Current selection display ---
        self.selected_path_display = QLineEdit()
        self.selected_path_display.setReadOnly(True)

        self.layout.addLayout(button_layout_top)
        self.layout.addWidget(self.selected_path_display)
        self.layout.addWidget(self.tree)
        self.layout.addLayout(button_layout)
        self.layout.addWidget(self.breadcrumb_widget)
        self.layout.addWidget(self.play_btn)

        # Connect buttons
        self.new_exp_btn.clicked.connect(self.create_new_experiment)
        self.rename_btn.clicked.connect(self.rename_file)
        self.delete_btn.clicked.connect(self.delete_file)
        self.save_btn.clicked.connect(self.save_file)
        self.play_btn.clicked.connect(self.play_traj)

        self.tree.selectionModel().selectionChanged.connect(self.update_buttons)

        self.update_buttons()
        self.update_breadcrumb(str(self.data_root))

        self.tree.setSortingEnabled(True)
        self.model.sort(0, Qt.AscendingOrder)

        self.tree.setEditTriggers(QAbstractItemView.EditKeyPressed | QAbstractItemView.SelectedClicked)
        self.model.setReadOnly(False)

        self.rename_shortcut = QShortcut(QKeySequence("F2"), self)
        self.rename_shortcut.activated.connect(self.rename_file)

        self.delete_shortcut = QShortcut(QKeySequence("Delete"), self)
        self.delete_shortcut.activated.connect(self.delete_file)

        self.new_experiment_shortcut = QShortcut(QKeySequence("Ctrl+Shift+N"), self)
        self.new_experiment_shortcut.activated.connect(self.create_new_experiment)

        self.play_traj_shortcut = QShortcut(QKeySequence("Enter"), self)
        self.play_traj_shortcut.activated.connect(self.play_traj)

        self.preload_all_experiment_configs()

    def file_selected(self, index):
        file_path = self.model.filePath(index)
        self.selected_path_display.setText(file_path)
        folder_path = os.path.dirname(file_path)
        self.update_breadcrumb(folder_path)

    def get_selected_path(self):
        index = self.tree.currentIndex()
        if not index.isValid():
            return None

        file_path = self.model.filePath(index)
        self.selected_path_display.setText(file_path)

        folder_path = os.path.dirname(file_path)
        self.update_breadcrumb(folder_path)

        return file_path

    def create_new_experiment(self):
        name, ok = QInputDialog.getText(self, "New Experiment", "Enter experiment folder name:")
        if not ok or not name:
            return

        if any(c in name for c in ('/', '\\')):
            QMessageBox.warning(self, "Invalid Name", "Folder name cannot contain slashes or backslashes.")
            return

        new_folder = os.path.join(str(self.data_root), name)
        try:
            os.makedirs(new_folder, exist_ok=False)

            package_root = Path(__file__).parent.parent
            config_path = package_root / "configs" / "temp.json"

            dst = os.path.join(new_folder, f'metadata.json')
            shutil.copyfile(config_path, dst)

            self.model.layoutChanged.emit()  # Force UI refresh
            QMessageBox.information(self, "Success", f"Experiment created at:\n{new_folder}")
        except FileExistsError:
            QMessageBox.warning(self, "Exists", "This experiment folder already exists.")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Could not create experiment folder:\n{e}")

    def rename_file(self):
        index = self.tree.currentIndex()
        if not index.isValid():
            return

        self.tree.edit(index)

    def delete_file(self):
        path = self.get_selected_path()
        if not path:
            return

        reply = QMessageBox.question(self, "Delete", f"Delete: {path}?", QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            try:
                if os.path.isdir(path):
                    shutil.rmtree(path)  # Delete directories
                else:
                    os.remove(path)  # Delete files
                self.model.layoutChanged.emit()  # Force UI refresh
                QMessageBox.information(self, "Deleted", f"Deleted: {path}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to delete:\n{e}")

    def save_file(self):
        """Unified Save/Save As functionality."""
        # Default to data_root
        initial_dir = str(self.data_root)
        suggested_filename = "new_trajectory.csv"

        selected_path = self.get_selected_path()

        if selected_path:
            if os.path.isdir(selected_path):
                initial_dir = selected_path
            else:
                initial_dir = os.path.dirname(selected_path)
                base_name = os.path.splitext(os.path.basename(selected_path))[0]
                suggested_filename = f"{base_name}.csv"

        # Ask where to save
        dialog = QFileDialog(self, "Save File", initial_dir, "CSV Files (*.csv)")
        dialog.setAcceptMode(QFileDialog.AcceptSave)
        dialog.setDefaultSuffix('csv')
        dialog.selectFile(suggested_filename)

        if dialog.exec_() == QFileDialog.Accepted:
            path = dialog.selectedFiles()[0]
            if not path.lower().endswith('.csv'):
                path += '.csv'

            try:
                if not os.path.exists(self.trajectory_logger.buffer_path):
                    raise FileNotFoundError(f"Buffer file not found at {self.trajectory_logger.buffer_path}")

                shutil.copy(str(self.trajectory_logger.buffer_path), path)
                self.model.layoutChanged.emit()

                QMessageBox.information(self, "Saved", f"Saved to:\n{path}")
            except Exception as e:
                QMessageBox.critical(self, "Error", str(e))

    def update_buttons(self):
        selected = self.tree.currentIndex().isValid()
        has_buffer = os.path.exists(self.trajectory_logger.buffer_path)

        self.rename_btn.setEnabled(selected)
        self.delete_btn.setEnabled(selected)
        self.save_btn.setEnabled(has_buffer)

    def update_breadcrumb(self, path):
        # Clear existing breadcrumb
        for i in reversed(range(self.breadcrumb.count())):
            widget = self.breadcrumb.itemAt(i).widget()
            if widget:
                self.breadcrumb.removeWidget(widget)
                widget.deleteLater()

        # Get absolute paths
        abs_path = Path(path).resolve()
        abs_root = Path(str(self.data_root)).resolve()

        # Only show path components that are within or at the data_root
        if abs_root in abs_path.parents or abs_path == abs_root:
            # Calculate relative path from root
            if abs_path == abs_root:
                # Just show the root name
                relative_parts = [abs_root.name]
                paths = [str(abs_root)]
            else:
                # Get path from root to current directory
                relative_path = abs_path.relative_to(abs_root.parent)
                relative_parts = relative_path.parts

                # Build list of paths
                paths = []
                current = abs_root.parent
                for part in relative_parts:
                    current = current / part
                    paths.append(str(current))
        else:
            # If path is outside data_root, just show the current directory
            relative_parts = [abs_path.name]
            paths = [str(abs_path)]

        # Create buttons for each part of the path
        for i, (part, path_str) in enumerate(zip(relative_parts, paths)):
            btn = QPushButton(part + "/")
            btn.setFlat(True)
            btn.setStyleSheet("text-decoration: underline; color: blue;")

            # Connect button click to navigation
            btn.clicked.connect(lambda _, p=path_str: self.tree.setRootIndex(self.model.index(p)))

            self.breadcrumb.addWidget(btn)

    def show_context_menu(self, position):
        index = self.tree.indexAt(position)
        if not index.isValid():
            return

        path = self.model.filePath(index)

        menu = QMenu(self)

        # Actions based on selection
        if os.path.isdir(path):
            new_folder_action = menu.addAction("New Folder", lambda: self.create_new_folder(path))
            new_folder_action.setIcon(self.style().standardIcon(QStyle.SP_DirOpenIcon))

            save_action = menu.addAction("Save here", self.save_file)
            save_action.setIcon(self.style().standardIcon(QStyle.SP_DialogSaveButton))

        rename_action = menu.addAction("Rename", self.rename_file_at)
        rename_action.setIcon(self.style().standardIcon(QStyle.SP_FileIcon))

        delete_action = menu.addAction("Delete", lambda: self.delete_file_at(path))
        delete_action.setIcon(self.style().standardIcon(QStyle.SP_TrashIcon))

        menu.exec_(self.tree.viewport().mapToGlobal(position))

    def create_new_folder(self, parent_dir):
        name, ok = QInputDialog.getText(self, "New Folder", "Folder name:")
        if ok and name:
            new_folder = os.path.join(parent_dir, name)
            try:
                os.makedirs(new_folder, exist_ok=False)
                self.model.layoutChanged.emit()
                QMessageBox.information(self, "Created", f"Created folder:\n{new_folder}")
            except FileExistsError:
                QMessageBox.warning(self, "Exists", "Folder already exists.")
            except Exception as e:
                QMessageBox.critical(self, "Error", str(e))

    def rename_file_at(self):
        index = self.tree.currentIndex()
        if not index.isValid():
            return

        self.tree.edit(index)

    def delete_file_at(self, path):
        reply = QMessageBox.question(self, "Delete", f"Delete: {path}?", QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            try:
                if os.path.isdir(path):
                    shutil.rmtree(path)
                else:
                    os.remove(path)
                self.model.layoutChanged.emit()
                QMessageBox.information(self, "Deleted", f"Deleted:\n{path}")
            except Exception as e:
                QMessageBox.critical(self, "Error", str(e))

    def preload_all_experiment_configs(self):
        if not hasattr(self, 'experiment_config_cache'):
            self.experiment_config_cache = {}

        for entry in os.listdir(str(self.data_root)):
            experiment_dir = os.path.join(str(self.data_root), entry)
            if os.path.isdir(experiment_dir):
                metadata_path = os.path.join(experiment_dir, 'metadata.json')
                metadata_path = os.path.abspath(metadata_path)
                if os.path.isfile(metadata_path) and metadata_path not in self.experiment_config_cache:
                    try:
                        with open(metadata_path, 'r') as f:
                            cfg = json.load(f)
                        self.experiment_config_cache[metadata_path] = cfg
                    except Exception as e:
                        print(f"Warning: Failed to preload {metadata_path}: {e}")

    def load_experiment_config(self, experiment_path):
        experiment_name = os.path.basename(experiment_path.rstrip(os.sep))
        metadata_path = os.path.join(str(self.data_root), experiment_name, 'metadata.json')
        metadata_path = os.path.abspath(metadata_path)

        # If already cached
        if metadata_path in self.experiment_config_cache:
            return self.experiment_config_cache[metadata_path]

        # If not cached, attempt to load dynamically
        if not os.path.isfile(metadata_path):
            QMessageBox.warning(self, "Missing Metadata", f"No metadata.json found for:\n{experiment_name}")
            return None

        try:
            with open(metadata_path, 'r') as f:
                cfg = json.load(f)
            self.experiment_config_cache[metadata_path] = cfg  # Dynamically add to cache
            return cfg
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load config for:\n{experiment_name}\n\nError: {e}")
            return None

    def find_experiment_root(self, start_path):
        current_path = os.path.abspath(start_path)

        while True:
            metadata_path = os.path.join(current_path, 'metadata.json')
            if os.path.isfile(metadata_path):
                return current_path  # Found the experiment root
            parent_path = os.path.dirname(current_path)
            if parent_path == current_path:
                # Reached filesystem root
                return None
            current_path = parent_path

    def play_traj(self):
        selected_path = self.get_selected_path()

        if not selected_path:
            return

        experiment_root = self.find_experiment_root(selected_path)
        if not experiment_root:
            QMessageBox.warning(self, "Experiment Not Found",
                                "Could not locate experiment root (metadata.json missing).")
            return

        cfg_loaded = self.load_experiment_config(experiment_root)
        if not cfg_loaded:
            return

        # Now read CSV
        try:
            # Assume the selected file is a CSV
            if not selected_path.endswith('.csv'):
                QMessageBox.warning(self, "Invalid Selection", "Please select a trajectory CSV file.")
                return

            df = pd.read_csv(selected_path)

            # Expect the CSV to have columns t, phi_arc, theta_arc
            t = df['t'].to_numpy()
            theta_arc = df['theta'].to_numpy()
            phi_arc = df['phi'].to_numpy()

            if all(np.array([len(t.tolist()) > 0, len(phi_arc.tolist()) > 0, len(theta_arc.tolist()) > 0])):
                self.trajectory_logger.empty_file = False
            else:
                self.trajectory_logger.empty_file = True

        except Exception as e:
            QMessageBox.critical(self, "CSV Error", f"Failed to load trajectory CSV:\n{e}")
            return

        # Prepare the result: constants + arrays
        results = (constants(cfg=cfg_loaded), t, phi_arc, theta_arc)

        # Emit results
        self.traj_constants_ready.emit(results)
