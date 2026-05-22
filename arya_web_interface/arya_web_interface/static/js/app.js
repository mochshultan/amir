/**
 * Main Application Initialization
 * Bootstraps all modules and starts the web control interface
 */

(function () {
  "use strict";

  // Initialize core modules
  const apiClient = new window.APIClient();
  const stateManager = window.stateManager;
  const uiRenderer = window.uiRenderer;
  const eventHandlers = new window.EventHandlers(
    stateManager,
    uiRenderer,
    apiClient
  );

  // Set initial status
  uiRenderer.setStatus("idle", "Connecting...");

  // Subscribe to state changes for UI updates
  stateManager.subscribe("selectedSection", (newSection) => {
    uiRenderer.showSection(newSection);
  });

  stateManager.subscribe("connected", (isConnected) => {
    if (isConnected) {
      uiRenderer.setStatus("connected", "Connected");
      // Load initial data
      eventHandlers.loadMaps();
    } else {
      uiRenderer.setStatus("disconnected", "Disconnected");
    }
  });

  stateManager.subscribe("*", (newState) => {
    // Update dashboard on any state change
    uiRenderer.updateDashboard(newState);
  });

  // Subscribe to error state for notifications
  stateManager.subscribe("lastError", (error) => {
    if (error) {
      uiRenderer.showAlert(error.message, "error", 5000);
      uiRenderer.announce(error.message, "error");
    }
  });

  // Export global app interface
  window.app = {
    state: stateManager,
    ui: uiRenderer,
    api: apiClient,
    events: eventHandlers,
    
    // Public API
    selectMap: (mapName) => eventHandlers.selectMap(mapName),
    loadMaps: () => eventHandlers.loadMaps(),
    getState: (key) => stateManager.getState(key),
    setState: (updates) => stateManager.setState(updates),
    showAlert: (msg, type, dur) => uiRenderer.showAlert(msg, type, dur),
  };

  // Log initialization
  console.log("[App]", "ARYA Web Interface initialized");
  console.log("[App]", "Available globally: window.app");

  // Initial health check
  apiClient.healthz().then((data) => {
    stateManager.setState({
      connected: data.ok,
      robotReady: data.ros_ready,
    });
  });
})();
