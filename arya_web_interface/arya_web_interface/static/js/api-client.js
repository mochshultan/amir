/**
 * API Client Module - HTTP communication with backend
 * Handles all REST API calls with error handling and retry logic
 */

class APIClient {
  constructor(baseURL = "/api") {
    this.baseURL = baseURL;
    this.timeout = 30000; // 30 seconds
  }

  /**
   * Perform HTTP request with built-in timeout and error handling
   * @param {string} endpoint - API endpoint path
   * @param {object} options - Fetch options (method, body, headers, etc)
   * @returns {Promise<object>} - Response JSON or error object
   */
  async request(endpoint, options = {}) {
    const {
      method = "GET",
      body = null,
      headers = {},
      signal = null,
      retries = 1,
    } = options;

    const url = `${this.baseURL}${endpoint}`;
    const controller = signal ? null : new AbortController();
    const timeoutHandle = setTimeout(
      () => (controller || signal)?.abort?.(),
      this.timeout
    );

    try {
      const fetchOptions = {
        method,
        headers: {
          "Content-Type": "application/json",
          ...headers,
        },
        signal: signal || controller?.signal,
      };

      if (body) {
        fetchOptions.body = JSON.stringify(body);
      }

      const response = await fetch(url, fetchOptions);

      if (!response.ok) {
        throw new APIError(
          `HTTP ${response.status}: ${response.statusText}`,
          response.status
        );
      }

      return await response.json();
    } catch (error) {
      clearTimeout(timeoutHandle);
      
      if (error.name === "AbortError") {
        throw new APIError("Request timeout", 408);
      }

      if (retries > 0 && error.code === "ECONNREFUSED") {
        return this.request(endpoint, { ...options, retries: retries - 1 });
      }

      throw error;
    }
  }

  // API Endpoint Methods

  async getMaps() {
    return this.request("/maps");
  }

  async selectMap(mapName) {
    return this.request("/maps/select", {
      method: "POST",
      body: { map_name: mapName },
    });
  }

  async getMapGrid(mapName) {
    return this.request(`/maps/grid?map_name=${encodeURIComponent(mapName)}`);
  }

  async getNavAnnotations(mapName) {
    return this.request(
      `/nav_annotations?map_name=${encodeURIComponent(mapName)}`
    );
  }

  async saveNavAnnotations(mapName, zones, stations) {
    return this.request("/nav_annotations", {
      method: "POST",
      body: { map_name: mapName, zones, stations },
    });
  }

  async getTopics() {
    return this.request("/topics");
  }

  async healthz() {
    return this.request("/healthz", { headers: {} }).catch(() => ({
      ok: false,
    }));
  }
}

// Custom Error Class
class APIError extends Error {
  constructor(message, code = 0) {
    super(message);
    this.name = "APIError";
    this.code = code;
  }
}

// Export as global or module
window.APIClient = APIClient;
window.APIError = APIError;
