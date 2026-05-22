/**
 * Event Handlers - User interaction and system events
 * Coordinates UI, state, and API interactions
 */

class EventHandlers {
  constructor(stateManager, uiRenderer, apiClient) {
    this.state = stateManager;
    this.ui = uiRenderer;
    this.api = apiClient;
    this.init();
  }

  init() {
    this._setupNavigationHandlers();
    this._setupConnectionHandlers();
    this._setupMapHandlers();
    this._setupControlHandlers();
  }

  _setupNavigationHandlers() {
    // Mobile menu toggle
    const btnMenu = document.getElementById("btn-menu");
    if (btnMenu) {
      btnMenu.addEventListener("click", () => {
        const isOpen = document.getElementById("sidebar")?.classList.contains(
          "open"
        );
        this.ui.toggleSidebar(!isOpen);
      });
    }

    // Section navigation
    document.querySelectorAll(".nav-item").forEach((item) => {
      item.addEventListener("click", (e) => {
        e.preventDefault();
        const section = item.dataset.section;
        if (section) {
          this.state.setState({ selectedSection: section });
          this.ui.showSection(section);
        }
      });
    });
  }

  _setupConnectionHandlers() {
    // Health check loop
    setInterval(() => {
      this.api
        .healthz()
        .then((data) => {
          this.state.setState({
            connected: data.ok,
            robotReady: data.ros_ready,
          });
          this.ui.setStatus(
            data.ok ? "connected" : "disconnected",
            data.ros_ready
              ? "Connected & ROS Ready"
              : "Connected (ROS initializing)"
          );
        })
        .catch((error) => {
          this.state.setState({ connected: false });
          this.ui.setStatus("error", "Connection Lost");
          console.error("Health check failed:", error);
        });
    }, 5000);
  }

  _setupMapHandlers() {
    const btnSetGoal = document.getElementById("btn-set-goal");
    if (btnSetGoal) {
      btnSetGoal.addEventListener("click", () => {
        this.ui.showAlert(
          "Click on map to set goal (future implementation)",
          "info"
        );
      });
    }

    const btnCancelGoal = document.getElementById("btn-cancel-goal");
    if (btnCancelGoal) {
      btnCancelGoal.addEventListener("click", () => {
        this.ui.showAlert("Goal cancelled", "success");
      });
    }
  }

  _setupControlHandlers() {
    const modeSelector = document.getElementById("mode-selector");
    if (modeSelector) {
      modeSelector.addEventListener("change", (e) => {
        this.state.setState({ driveMode: e.target.value });
        this.ui.showAlert(
          `Drive mode changed to: ${e.target.value}`,
          "success",
          3000
        );
      });
    }

    // Joystick canvas interaction (future: add joystick lib)
    const canvasJoystick = document.getElementById("canvas-joystick");
    if (canvasJoystick) {
      canvasJoystick.addEventListener("mousedown", (e) => {
        // Future: implement joystick dragging
      });
    }
  }

  /**
   * Load and display maps
   */
  async loadMaps() {
    try {
      this.ui.showLoading(document.getElementById("dashboard-content"));
      const data = await this.api.getMaps();
      this.state.setState({
        availableMaps: data.maps,
        selectedMap: data.selected_map,
      });
      this.ui.showAlert(`Loaded ${data.maps.length} maps`, "info", 3000);
    } catch (error) {
      this.state.setError("Failed to load maps", error);
      this.ui.showAlert("Failed to load maps", "error");
    }
  }

  /**
   * Select and load a specific map
   */
  async selectMap(mapName) {
    try {
      await this.api.selectMap(mapName);
      this.state.setState({ selectedMap: mapName });
      this.ui.showAlert(`Map selected: ${mapName}`, "success", 3000);

      // Load map grid
      const gridData = await this.api.getMapGrid(mapName);
      this.state.setState({ mapGrid: gridData });
      this.ui.renderMapCanvas(
        document.getElementById("canvas-map"),
        gridData
      );
    } catch (error) {
      this.state.setError("Failed to select map", error);
      this.ui.showAlert("Failed to select map", "error");
    }
  }
}

// Export and auto-initialize
window.EventHandlers = EventHandlers;
