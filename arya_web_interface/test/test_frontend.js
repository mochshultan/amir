/**
 * Frontend Unit Tests
 * Run with: jest test/test-api-client.js
 */

describe("APIClient", () => {
  let client;

  beforeEach(() => {
    client = new window.APIClient();
    // Mock fetch
    global.fetch = jest.fn();
  });

  afterEach(() => {
    jest.resetAllMocks();
  });

  describe("GET requests", () => {
    test("should make GET request", async () => {
      global.fetch.mockResolvedValueOnce({
        ok: true,
        json: async () => ({ success: true }),
      });

      const result = await client.request("/test");
      expect(result).toEqual({ success: true });
      expect(global.fetch).toHaveBeenCalled();
    });

    test("should handle HTTP errors", async () => {
      global.fetch.mockResolvedValueOnce({
        ok: false,
        status: 404,
        statusText: "Not Found",
      });

      await expect(client.request("/not-found")).rejects.toThrow("HTTP 404");
    });

    test("should handle network errors", async () => {
      global.fetch.mockRejectedValueOnce(new Error("Network error"));

      await expect(client.request("/test")).rejects.toThrow();
    });
  });

  describe("POST requests", () => {
    test("should POST with body", async () => {
      global.fetch.mockResolvedValueOnce({
        ok: true,
        json: async () => ({ created: true }),
      });

      const result = await client.request("/create", {
        method: "POST",
        body: { name: "test" },
      });

      expect(result).toEqual({ created: true });
      expect(global.fetch).toHaveBeenCalledWith(
        expect.any(String),
        expect.objectContaining({
          method: "POST",
          body: JSON.stringify({ name: "test" }),
        })
      );
    });
  });

  describe("API endpoints", () => {
    test("getMaps should call correct endpoint", async () => {
      global.fetch.mockResolvedValueOnce({
        ok: true,
        json: async () => ({ maps: ["map1.yaml"] }),
      });

      await client.getMaps();
      expect(global.fetch).toHaveBeenCalledWith(
        expect.stringContaining("/api/maps"),
        expect.any(Object)
      );
    });

    test("selectMap should POST map selection", async () => {
      global.fetch.mockResolvedValueOnce({
        ok: true,
        json: async () => ({ ok: true }),
      });

      await client.selectMap("test.yaml");
      expect(global.fetch).toHaveBeenCalledWith(
        expect.any(String),
        expect.objectContaining({ method: "POST" })
      );
    });
  });
});

describe("StateManager", () => {
  let manager;

  beforeEach(() => {
    manager = new window.StateManager();
  });

  test("should initialize with default state", () => {
    expect(manager.getState("connected")).toBe(false);
    expect(manager.getState("driveMode")).toBe("manual");
  });

  test("should update state", () => {
    manager.setState({ connected: true, driveMode: "auto" });
    expect(manager.getState("connected")).toBe(true);
    expect(manager.getState("driveMode")).toBe("auto");
  });

  test("should notify subscribers on state change", (done) => {
    manager.subscribe("connected", (newValue, oldValue) => {
      expect(newValue).toBe(true);
      expect(oldValue).toBe(false);
      done();
    });

    manager.setState({ connected: true });
  });

  test("should support dot notation for nested state", () => {
    manager.setState({ odom: [1, 2, 3] });
    expect(manager.getState("odom.0")).toBe(1);
  });

  test("should report errors", () => {
    manager.setError("Test error");
    expect(manager.getState("lastError")).toBeTruthy();
    expect(manager.getState("lastError").message).toBe("Test error");
  });
});

describe("UIRenderer", () => {
  let renderer;

  beforeEach(() => {
    // Setup DOM
    document.body.innerHTML = `
      <div id="app">
        <div id="sidebar"></div>
        <div id="status-indicator"></div>
        <div id="status-text"></div>
        <div id="alerts"></div>
        <div id="dashboard-content"></div>
        <canvas id="canvas-map"></canvas>
        <canvas id="canvas-joystick"></canvas>
        <select id="mode-selector"></select>
      </div>
    `;
    renderer = new window.UIRenderer();
  });

  test("should cache DOM elements", () => {
    expect(renderer.elements.app).toBeTruthy();
    expect(renderer.elements.sidebar).toBeTruthy();
  });

  test("should set status", () => {
    renderer.setStatus("connected", "Connected");
    expect(document.getElementById("status-text").textContent).toBe(
      "Connected"
    );
  });

  test("should show alerts", () => {
    renderer.showAlert("Test alert", "info");
    expect(document.querySelector(".alert")).toBeTruthy();
  });

  test("should announce for screen readers", (done) => {
    renderer.announce("Announcement");
    setTimeout(() => {
      expect(document.querySelector("[role='status']")).toBeTruthy();
      done();
    }, 50);
  });

  test("should render map on canvas", () => {
    const canvas = document.getElementById("canvas-map");
    const gridData = {
      w: 10,
      h: 10,
      b64: btoa(String.fromCharCode(...new Array(100).fill(128))),
    };

    renderer.renderMapCanvas(canvas, gridData);
    expect(canvas.width).toBe(10);
    expect(canvas.height).toBe(10);
  });

  test("should toggle sidebar", () => {
    renderer.toggleSidebar(true);
    expect(document.getElementById("sidebar").classList.contains("open")).toBe(
      true
    );

    renderer.toggleSidebar(false);
    expect(document.getElementById("sidebar").classList.contains("open")).toBe(
      false
    );
  });
});
