/**
 * State Manager - Centralized application state management
 * Uses pub/sub pattern for state change notifications
 */

class StateManager {
  constructor() {
    this.state = {
      // Robot state
      connected: false,
      robotReady: false,
      driveMode: "manual", // manual | auto
      telemetry: [0, 0, 0, 0, 0, 0],
      odom: [0, 0, 0],
      amclPose: null,
      
      // Maps and navigation
      availableMaps: [],
      selectedMap: null,
      mapGrid: null,
      navAnnotations: null,
      
      // Navigation goal
      goalStatus: {
        state: "idle",
        message: "No active goal",
        seq: 0,
      },
      
      // Mission queue
      missionStatus: {
        state: "idle",
        message: "No active mission",
        mode: "idle",
        mission_id: 0,
        current_index: -1,
        total: 0,
        station: null,
      },
      
      // UI state
      selectedSection: "dashboard",
      sidebarOpen: false,
      darkMode: this._prefersDarkMode(),
      
      // Error state
      lastError: null,
      lastErrorTime: 0,
    };

    // Subscribers registry
    this.subscribers = {};
  }

  /**
   * Subscribe to state changes
   * @param {string} key - State property to watch
   * @param {function} callback - Called with (newValue, oldValue) when key changes
   * @returns {function} - Unsubscribe function
   */
  subscribe(key, callback) {
    if (!this.subscribers[key]) {
      this.subscribers[key] = [];
    }
    this.subscribers[key].push(callback);

    // Return unsubscribe function
    return () => {
      this.subscribers[key] = this.subscribers[key].filter(
        (cb) => cb !== callback
      );
    };
  }

  /**
   * Update state and notify subscribers
   * @param {object} updates - Partial state updates
   */
  setState(updates) {
    const oldState = { ...this.state };
    this.state = { ...this.state, ...updates };

    // Notify subscribers for changed keys
    Object.keys(updates).forEach((key) => {
      if (this.subscribers[key]) {
        this.subscribers[key].forEach((callback) => {
          callback(this.state[key], oldState[key]);
        });
      }
    });

    // Notify global subscribers
    if (this.subscribers["*"]) {
      this.subscribers["*"].forEach((callback) => {
        callback(this.state, oldState);
      });
    }
  }

  /**
   * Get current state value
   * @param {string} key - State property path (supports dot notation)
   * @returns {any} - Current value
   */
  getState(key) {
    const parts = key.split(".");
    let value = this.state;
    for (const part of parts) {
      value = value?.[part];
    }
    return value;
  }

  /**
   * Report error to state
   * @param {string} message - Error message
   * @param {Error} error - Optional error object
   */
  setError(message, error = null) {
    this.setState({
      lastError: { message, error, timestamp: Date.now() },
      lastErrorTime: Date.now(),
    });
    console.error("[StateManager]", message, error);
  }

  /**
   * Clear error state
   */
  clearError() {
    this.setState({ lastError: null });
  }

  _prefersDarkMode() {
    return window.matchMedia?.("(prefers-color-scheme: dark)").matches ?? false;
  }

  /**
   * Export state for debugging
   * @returns {object} - Current state snapshot
   */
  dump() {
    return { ...this.state };
  }
}

// Export as global singleton
window.stateManager = new StateManager();
