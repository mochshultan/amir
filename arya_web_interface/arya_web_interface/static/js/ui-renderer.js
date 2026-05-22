/**
 * UI Renderer - DOM manipulation and component rendering
 * Decoupled from event handling and API logic
 */

class UIRenderer {
  constructor() {
    this.elements = this._cacheElements();
  }

  /**
   * Cache frequently accessed DOM elements
   * @private
   */
  _cacheElements() {
    return {
      app: document.getElementById("app"),
      sidebar: document.getElementById("sidebar"),
      btnMenu: document.getElementById("btn-menu"),
      statusIndicator: document.getElementById("status-indicator"),
      statusText: document.getElementById("status-text"),
      alertsContainer: document.getElementById("alerts"),
      contentArea: document.getElementById("content-area"),
      dashboardContent: document.getElementById("dashboard-content"),
      canvasMap: document.getElementById("canvas-map"),
      canvasJoystick: document.getElementById("canvas-joystick"),
      modeSelector: document.getElementById("mode-selector"),
    };
  }

  /**
   * Update status indicator
   * @param {string} status - 'connected' | 'disconnected' | 'error' | 'idle'
   * @param {string} message - Status text
   */
  setStatus(status, message) {
    const indicator = this.elements.statusIndicator;
    const text = this.elements.statusText;
    
    if (!indicator || !text) return;

    // Clear previous status classes
    indicator.className = "status-led";
    
    // Apply status class
    if (status === "connected") {
      indicator.classList.add("connected");
    } else if (status === "error") {
      indicator.classList.add("error");
    }

    text.textContent = message;
  }

  /**
   * Show alert/toast notification
   * @param {string} message - Alert message
   * @param {string} type - 'success' | 'error' | 'warning' | 'info'
   * @param {number} duration - Auto-dismiss duration in ms (0 = manual only)
   */
  showAlert(message, type = "info", duration = 5000) {
    const container = this.elements.alertsContainer;
    if (!container) return;

    const alert = document.createElement("div");
    alert.className = `alert alert-${type}`;
    alert.setAttribute("role", "status");
    alert.setAttribute("aria-live", "polite");
    alert.textContent = message;

    container.appendChild(alert);

    if (duration > 0) {
      setTimeout(() => alert.remove(), duration);
    }

    return () => alert.remove();
  }

  /**
   * Toggle sidebar visibility (mobile)
   * @param {boolean} open - True to open, false to close
   */
  toggleSidebar(open) {
    const sidebar = this.elements.sidebar;
    if (!sidebar) return;

    if (open === undefined) {
      open = !sidebar.classList.contains("open");
    }

    if (open) {
      sidebar.classList.add("open");
    } else {
      sidebar.classList.remove("open");
    }
  }

  /**
   * Switch to content section
   * @param {string} sectionId - Section identifier
   */
  showSection(sectionId) {
    // Hide all sections
    document.querySelectorAll(".content-section").forEach((section) => {
      section.classList.remove("active");
    });

    // Show selected section
    const section = document.getElementById(`section-${sectionId}`);
    if (section) {
      section.classList.add("active");
    }

    // Update nav items
    document.querySelectorAll(".nav-item").forEach((item) => {
      item.classList.remove("active");
      if (item.dataset.section === sectionId) {
        item.classList.add("active");
      }
    });

    // Close sidebar on mobile
    if (window.innerWidth <= 768) {
      this.toggleSidebar(false);
    }
  }

  /**
   * Update dashboard cards with telemetry data
   * @param {object} data - Telemetry data object
   */
  updateDashboard(data) {
    const container = this.elements.dashboardContent;
    if (!container) return;

    // Build dashboard cards HTML
    const html = `
      <div class="card">
        <h3 class="card-title">Robot Status</h3>
        <div class="card-content">
          <p>Connected: ${data.connected ? "Yes" : "No"}</p>
          <p>Drive Mode: <strong>${data.driveMode}</strong></p>
        </div>
      </div>
      <div class="card">
        <h3 class="card-title">Position</h3>
        <div class="card-content">
          <p>X: ${data.odom?.[0]?.toFixed(3) || "N/A"} m</p>
          <p>Y: ${data.odom?.[1]?.toFixed(3) || "N/A"} m</p>
          <p>θ: ${data.odom?.[2]?.toFixed(3) || "N/A"} rad</p>
        </div>
      </div>
      <div class="card">
        <h3 class="card-title">Navigation</h3>
        <div class="card-content">
          <p>Goal State: <strong>${data.goalStatus?.state || "idle"}</strong></p>
          <p>${data.goalStatus?.message || "No active goal"}</p>
        </div>
      </div>
    `;

    container.innerHTML = html;
  }

  /**
   * Render map visualization on canvas
   * @param {HTMLCanvasElement} canvas - Canvas element
   * @param {object} gridData - OccupancyGrid data
   */
  renderMapCanvas(canvas, gridData) {
    if (!canvas || !gridData) return;

    const ctx = canvas.getContext("2d");
    const { w, h, b64 } = gridData;

    canvas.width = w;
    canvas.height = h;

    // Decode base64 grid data
    const binaryString = atob(b64);
    const imageData = ctx.createImageData(w, h);
    const data = imageData.data;

    for (let i = 0; i < binaryString.length; i++) {
      const value = binaryString.charCodeAt(i);
      const pixelIndex = i * 4;

      // Convert occupancy (0-100/-1) to RGB
      let gray;
      if (value === 255 || value < 0) {
        gray = 128; // Unknown (gray)
      } else if (value === 0) {
        gray = 255; // Free (white)
      } else if (value === 100) {
        gray = 0; // Occupied (black)
      } else {
        gray = 255 - Math.round((value / 100) * 255); // Scale between white and black
      }

      data[pixelIndex] = gray;
      data[pixelIndex + 1] = gray;
      data[pixelIndex + 2] = gray;
      data[pixelIndex + 3] = 255; // Alpha
    }

    ctx.putImageData(imageData, 0, 0);
  }

  /**
   * Update joystick visualization
   * @param {HTMLCanvasElement} canvas - Canvas element
   * @param {number} x - X position (-1 to 1)
   * @param {number} y - Y position (-1 to 1)
   */
  updateJoystickViz(canvas, x, y) {
    if (!canvas) return;

    const ctx = canvas.getContext("2d");
    const width = canvas.width || 200;
    const height = canvas.height || 200;
    const centerX = width / 2;
    const centerY = height / 2;
    const radius = Math.min(width, height) / 2 - 10;

    // Clear
    ctx.fillStyle = "rgba(0, 0, 0, 0.05)";
    ctx.fillRect(0, 0, width, height);

    // Draw outer circle
    ctx.strokeStyle = "rgba(0, 100, 200, 0.3)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(centerX, centerY, radius, 0, Math.PI * 2);
    ctx.stroke();

    // Draw center point
    ctx.fillStyle = "rgba(100, 150, 255, 0.1)";
    ctx.beginPath();
    ctx.arc(centerX, centerY, 5, 0, Math.PI * 2);
    ctx.fill();

    // Draw stick position
    const stickX = centerX + x * radius;
    const stickY = centerY - y * radius; // Inverted Y
    ctx.fillStyle = "rgba(10, 110, 209, 0.8)";
    ctx.beginPath();
    ctx.arc(stickX, stickY, 10, 0, Math.PI * 2);
    ctx.fill();
  }

  /**
   * Clear and show loading state
   * @param {HTMLElement} element - Element to show loading in
   */
  showLoading(element) {
    if (!element) return;
    element.innerHTML = '<div class="spinner"></div> Loading...';
  }

  /**
   * Generic notification with ARIA live region support
   * @param {string} message - Message text
   * @param {string} level - 'info' | 'warning' | 'error'
   */
  announce(message, level = "info") {
    const region = document.createElement("div");
    region.className = "sr-only";
    region.setAttribute("role", "status");
    region.setAttribute("aria-live", level === "error" ? "assertive" : "polite");
    region.textContent = message;

    document.body.appendChild(region);
    setTimeout(() => region.remove(), 3000);
  }
}

// Export as global singleton
window.uiRenderer = new UIRenderer();
