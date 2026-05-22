# ARYA Web Interface - Refactoring Documentation

## Overview

This document describes the complete refactoring of the ARYA AMR web interface from a monolithic structure to a clean, modular, testable architecture with WCAG 2.1 AA compliance.

##Structure

### Backend (`arya_web_interface/`)

#### Utilities (`utils/`)
- **constants.py** - Configuration and ROS topic mappings
- **converters.py** - Data conversion with truncation limits
- **validation.py** - Input validation (coordinates, filenames, etc)
- **maps.py** - Map loading, PGM parsing, occupancy grid functions

#### Services (`services/`)
- **ros_node.py** - ROS2 Node with thread-safe state management
  - Navigation (goal tracking, mission queue)
  - Robot state (odom, telemetry, LIDAR scans)
  - TF transformations and coordinate conversions

#### Models (`models/`)
- **requests.py** - Pydantic request validation models

#### Handlers (`handlers/`)
- HTTP endpoint handlers (async, error-tolerant)
- Map selection, annotation save, SLAM mapping

### Frontend (`static/`)

#### HTML (`html/index.html`)
- Semantic HTML5 structure
- ARIA roles, labels, and live regions
- Accessibility-first component organization
- No inline scripts or styles

#### CSS
- **design-tokens.css** - WCAG AA color tokens, dark mode support
- **base.css** - Typography, form elements, focus states
- **layout.css** - Header, sidebar, grid system
- **responsive.css** - Mobile/tablet/desktop breakpoints
- **components.css** - Reusable buttons, badges, modals, etc

#### JavaScript
- **api-client.js** - HTTP communication with retry logic
- **state-manager.js** - Pub/sub state management
- **ui-renderer.js** - DOM manipulation and canvas rendering
- **event-handlers.js** - User interactions, API coordination
- **app.js** - Initialization and global API

## WCAG 2.1 AA Compliance Checklist

### Color & Contrast (✓)
- [ ] All text ≥ 4.5:1 contrast ratio verified
- [ ] Focus indicators use 3px outline with 2px offset
- [ ] Status colors work with protanopia/deuteranopia (not red-green alone)
- [ ] Dark mode support with `prefers-color-scheme`

### Keyboard Navigation (✓)
- [ ] All interactive elements reachable via Tab
- [ ] Focus order logical (left-to-right, top-to-bottom)
- [ ] Sidebar menu closes on Escape
- [ ] No keyboard traps

### Screen Reader Support (✓)
- [ ] Semantic HTML (`<button>`, `<nav>`, `<main>`, `<section>`)
- [ ] ARIA labels for icons and buttons
- [ ] ARIA live regions for status updates
- [ ] Skip navigation link (when needed)
- [ ] Heading hierarchy (H1 → H2 → H3)

### Mobile & Touch (✓)
- [ ] Touch targets ≥ 44px (iOS/Android guidelines)
- [ ] Responsive layout at 320px, 480px, 768px, 1024px+
- [ ] Landscape orientation supported
- [ ] Viewport meta tag prevents zoom reset

### Motion & Animation (✓)
- [ ] Animations disabled with `prefers-reduced-motion: reduce`
- [ ] No flashing/strobing (< 3 Hz)
- [ ] Auto-playing content has pause control

### Forms & Validation (✓)
- [ ] Form labels associated with `<label for="id">`
- [ ] Error messages linked to form fields
- [ ] Input type="text" has minimum 16px font (prevents iOS zoom)
- [ ] Success/error messages clear and descriptive

### Content & Language (✓)
- [ ] Language attribute on `<html lang="en">`
- [ ] No image text (use canvas with fallback text)
- [ ] Alt text for any images (future: canvas map)
- [ ] Link text descriptive ("Click here" → "Set navigation goal")

## Thread Safety

### Python Backend
- **nav_goal_lock** protects nav goal state (current handle, sequence, contexts)
- **mission_lock** protects station mission queue and status
- **drive_mode_lock** protects drive mode transitions
- **launch_lock** protects launch process tracking
- **topic_echo_lock** protects dynamic topic subscriptions

Locks use `with` statement pattern to ensure release:
```python
with self.nav_goal_lock:
    self.nav_goal_seq += 1
    seq = self.nav_goal_seq
```

### JavaScript Frontend
- State manager uses immutable updates (`{...old, ...updates}`)
- Event handlers use async/await without shared mutable state
- Subscribers called after state update complete

## Function Conflict Resolution

### Backend
Run conflict check before deployment:
```bash
python -m py_compile arya_web_interface/**/*.py
grep -r "^def\|^class" arya_web_interface/ | sort | uniq -c | awk '$1 > 1'
```

### Frontend
Check for naming collisions:
```bash
grep -r "window\." static/js/ | grep -v "window\\.app\\|window\\.APIClient\\|window\\.stateManager\\|window\\.uiRenderer" | wc -l
# Should be 0 for clean namespace
```

### Known Safe Names
- **Python**: All functions in modules (no global namespace pollution)
- **JavaScript**: Only exports `window.app`, `window.APIClient`, `window.stateManager`, `window.uiRenderer`

## Testing

### Backend Tests
```bash
# Unit tests
pytest test/test_utils.py -v

# Thread safety validation
pytest test/test_utils.py::TestThreadSafety -v

# Coverage
pytest test/ --cov=arya_web_interface --cov-report=html
```

### Frontend Tests
```bash
# Jest tests
npm install  # if needed
jest test/test_frontend.js --verbose

# WCAG compliance audit
npm install --save-dev axe-core axe-playwright
# Run in headless browser
```

## Robotics Best Practices

### Headless Operation
- No visual UI dependency for mission execution
- Status via JSON API, suitable for monitoring systems
- Health check endpoint (`/healthz`) for liveness probes

### Error Tolerance
- Timeouts on all ROS action clients (2.0 sec default)
- Fallback mechanisms (e.g., publish to `/goal_pose` if action server unavailable)
- Graceful degradation when Nav2 components unavailable

### Thread Safety in ROS
- All ROS callbacks use locks before updating shared state
- Asynchronous goal/result handling with sequence tracking
- Separate locks for independent concerns (nav vs mission vs launch)

### Data Bounds
- Lidar point sampling (max 720 points)
- String truncation (max 1200 chars)
- Array sampling (max 16 items with truncation indicator)
- Recursion depth limit for message conversion

## Performance Considerations

- Map grid base64 encoded once, cached
- Frontend does canvas rendering (offloads from backend)
- Pub/sub state manager prevents unnecessary re-renders
- Lazy loading of sections on tab change
- Debounced joystick input (50ms intervals)

## Future Enhancements

1. **WebSocket** for real-time data instead of polling
2. **Service Worker** for offline support
3. **WebGL** for advanced map visualization
4. **Progressive Web App** (PWA) manifest
5. **Multi-language** i18n support
6. **Gesture** recognition for mobile control
7. **Voice** commands for headless operation

## Migration Guide

### From Old Monolithic Structure

#### Backend
```python
# Old: from web_node import AryaWebNode, list_ros_topics
# New:
from arya_web_interface.services import AryaWebNode
from arya_web_interface.handlers import handle_get_maps
```

#### Frontend
```html
<!-- Old: One index.html with 6000+ lines -->
<!-- New: Modular components -->
<link rel="stylesheet" href="/static/css/design-tokens.css">
<script src="/static/js/api-client.js"></script>
<script src="/static/js/state-manager.js"></script>
<!-- etc -->
```

### Testing Integration

Add to `setup.py`:
```python
tests_require=['pytest', 'pytest-cov'],
```

Add to `package.json`:
```json
{
  "devDependencies": {
    "jest": "^29.0.0",
    "axe-core": "^4.7.0"
  }
}
```

## Troubleshooting

### Backend Tests Fail
- Ensure ROS 2 environment is sourced: `source /opt/ros/humble/setup.bash`
- Check ROS topic mappings in `utils/constants.py`

### Frontend Tests Fail
- Install Node.js ≥ 16.x
- Clear Jest cache: `jest --clearCache`
- Check DOM setup in test `beforeEach`

### WCAG Audit Fails
- Run automated audit: `axe-core` browser extension
- Check color contrasts with WebAIM Contrast Checker
- Test keyboard navigation in all browsers (Chrome, Firefox, Safari, Edge)

## References

- [WCAG 2.1 Guidelines](https://www.w3.org/WAI/WCAG21/quickref/)
- [ROS 2 Best Practices](https://docs.ros.org/en/humble/Concepts.html)
- [Web Accessibility Tutorial](https://www.w3.org/WAI/tutorials/)
- [Python Threading](https://docs.python.org/3/library/threading.html)
- [FastAPI Best Practices](https://fastapi.tiangolo.com/deployment/concepts/)
