#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>   

/* ==================================
   SETTINGS
================================== */

// Your home WiFi (the ESP32 only supports 2.4 GHz networks)
const char* ssid = "laplace";
const char* password = "123";

// The ESP32 also creates its own WiFi network with this name (password needs 8+ characters)
const char* apName = "laplace";
const char* apPassword = "123";

// Pins (from the Wokwi diagram)
const int X_STEP = 33, X_DIR = 25;   // stepper1 / drv1 = X axis
const int Z_STEP = 17, Z_DIR = 26;   // stepper2 / drv2 = Z axis (claw up/down)
const int Y_STEP = 16, Y_DIR = 27;   // stepper3 / drv3 = Y axis
const int SERVO_PIN = 14;            // claw servo signal

// Claw servo angles (degrees) - adjust to your claw
const int CLAW_OPEN_ANGLE = 90;
const int CLAW_CLOSED_ANGLE = 0;

// Motion tuning
const int  STEPS_PER_PX   = 4;       // motor steps per grid pixel (grid = 450 px)
const long Z_TRAVEL_STEPS = 400;     // steps from "up" to "down"
const int  STEP_US        = 1000;    // half-period of a step pulse (smaller = faster)
const int  CLAW_MS        = 500;     // time the claw needs to open or close
const int  MAX_PX         = 450;

// Flip a value to 1 if an axis moves the wrong way
const int X_INVERT = 0, Y_INVERT = 0, Z_INVERT = 0;

/* ==================================
   STATE
================================== */

WebServer server(80);
Servo clawServo;

// Position in steps. The robot is assumed to start at the grid centre (225, 225), Z up.
volatile long posX = 225L * STEPS_PER_PX;
volatile long posY = 225L * STEPS_PER_PX;
volatile long posZ = 0;

volatile bool busy = false;
volatile bool abortFlag = false;

enum CmdType { CMD_MOVE, CMD_Z, CMD_CLAW_OPEN, CMD_CLAW_CLOSE };

struct Cmd {
  CmdType type;
  long a;
  long b;
};

QueueHandle_t cmdQueue;

/* ==================================
   WEB PAGE
================================== */

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Cartesian Robot Control</title>
<style>
* { box-sizing: border-box; }
body { margin: 0; min-height: 100vh; display: flex; justify-content: center; align-items: center; background: #f2f2f2; font-family: Arial, sans-serif; }
.container { display: flex; align-items: flex-start; gap: 15px; transform: translateX(-70px); }
.grid-container { text-align: center; }
#coordinates { width: 450px; font-size: 24px; font-weight: bold; margin-bottom: 4px; }
#cursor { height: 18px; margin-bottom: 8px; font-size: 13px; color: #888; }
#grid { box-sizing: content-box; width: 450px; height: 450px; border: 3px solid black; background-color: white;
  background-image: linear-gradient(#ccc 1px, transparent 1px), linear-gradient(90deg, #ccc 1px, transparent 1px);
  background-size: 25px 25px; position: relative; overflow: hidden; cursor: crosshair; }
#canvas { position: absolute; top: 0; left: 0; width: 100%; height: 100%; pointer-events: none; }
#status { width: 450px; margin-top: 12px; font-size: 14px; color: #666; }
.window { width: 260px; height: 560px; background: white; border: 2px solid #ccc; border-radius: 10px; padding: 18px; box-shadow: 0 3px 10px rgba(0,0,0,0.1); }
.window h2 { margin: 0 0 15px 0; }
#current-course-name { font-size: 18px; font-weight: bold; margin-bottom: 12px; padding: 10px; background: #f0f0f0; border-radius: 6px; }
.course-buttons { display: flex; gap: 6px; margin-bottom: 10px; }
.course-buttons button { flex: 1; padding: 10px 4px; border: none; border-radius: 6px; color: white; font-weight: bold; cursor: pointer; }
#run-course { background: #1976d2; }
#stop-course { background: #e53935; }
#delete-all { width: 100%; padding: 9px; margin-bottom: 15px; border: none; border-radius: 6px; background: #555; color: white; font-weight: bold; cursor: pointer; }
#run-course:hover { background: #1565c0; }
#stop-course:hover { background: #c62828; }
#delete-all:hover { background: #333; }
.list { max-height: 355px; overflow-y: auto; }
.point { border: 1px solid #ccc; border-left-width: 5px; border-radius: 7px; padding: 10px; margin-bottom: 10px; background: #fafafa; }
.point.pick { border-left-color: #9c27b0; }
.point.move { border-left-color: #e53935; }
.point.place { border-left-color: #ef6c00; }
.point-title { font-weight: bold; margin-bottom: 5px; }
.point-type { font-size: 12px; font-weight: bold; margin-bottom: 8px; }
.pick .point-type { color: #9c27b0; }
.move .point-type { color: #e53935; }
.place .point-type { color: #ef6c00; }
.point-coordinates { font-size: 14px; color: #555; margin-bottom: 8px; }
.goto-button, .delete-button { border: none; border-radius: 5px; padding: 6px 9px; cursor: pointer; }
.goto-button { background: #4CAF50; color: white; }
.delete-button { background: #e53935; color: white; margin-left: 5px; }
#new-course { width: 100%; padding: 11px; margin-bottom: 15px; border: none; border-radius: 6px; background: #1976d2; color: white; font-size: 15px; font-weight: bold; cursor: pointer; }
#new-course:hover { background: #1565c0; }
.course-item { display: flex; align-items: center; gap: 5px; padding: 10px; margin-bottom: 8px; border: 1px solid #ccc; border-radius: 7px; background: #fafafa; }
.course-item.selected { border: 2px solid #1976d2; background: #eaf3ff; }
.course-select { flex: 1; border: none; background: transparent; text-align: left; font-size: 15px; cursor: pointer; }
.course-delete { background: #e53935; color: white; border: none; border-radius: 5px; padding: 5px 8px; cursor: pointer; }
.coordinate-window { width: 220px; height: auto; }
.coordinate-inputs { display: flex; gap: 8px; margin-bottom: 10px; }
.coordinate-input { width: 50%; }
.coordinate-input label { display: block; font-size: 13px; font-weight: bold; margin-bottom: 4px; }
.coordinate-input input { width: 100%; padding: 8px; border: 1px solid #bbb; border-radius: 5px; font-size: 15px; }
.coordinate-window button { width: 100%; padding: 10px; border: none; border-radius: 6px; color: white; font-weight: bold; cursor: pointer; }
#move-coordinate { background: #4CAF50; }
#move-coordinate:hover { background: #388e3c; }
.add-buttons { display: flex; flex-direction: column; gap: 6px; margin-top: 10px; }
#add-pick { background: #9c27b0; }
#add-move { background: #e53935; }
#add-place { background: #ef6c00; }
.help { margin-top: 12px; font-size: 12px; line-height: 1.5; color: #777; }
.empty { color: #888; font-size: 14px; text-align: center; padding: 20px 5px; }
button:disabled { background: #999 !important; cursor: not-allowed; }
</style>
</head>
<body>

<div class="container">

  <div class="grid-container">
    <div id="coordinates">X: 225 | Y: 225 | Z: 0%</div>
    <div id="cursor"></div>
    <div id="grid"><canvas id="canvas"></canvas></div>
    <div id="status">Left-click = pick | Middle-click = move | Right-click = place</div>
  </div>

  <div class="window">
    <h2>Current Course</h2>
    <div id="current-course-name">Course 1</div>
    <div class="course-buttons">
      <button id="run-course" onclick="runCourse()">Run Course</button>
      <button id="stop-course" onclick="stopCourse()">Stop</button>
    </div>
    <button id="delete-all" onclick="deleteAllPoints()">Delete All Points</button>
    <div id="current-points" class="list"></div>
  </div>

  <div class="window">
    <h2>Saved Courses</h2>
    <button id="new-course" onclick="createCourse()">+ New Course</button>
    <div id="course-list" class="list"></div>
  </div>

  <div class="window coordinate-window">
    <h2>Manual Move</h2>
    <div class="coordinate-inputs">
      <div class="coordinate-input">
        <label for="input-x">X</label>
        <input id="input-x" type="number" min="0" max="450" value="225">
      </div>
      <div class="coordinate-input">
        <label for="input-y">Y</label>
        <input id="input-y" type="number" min="0" max="450" value="225">
      </div>
    </div>
    <button id="move-coordinate" onclick="moveToCoordinates()">Move To Coordinates</button>
    <div class="add-buttons">
      <button id="add-pick" onclick="addPointFromInputs('pick')">Add Pick Point</button>
      <button id="add-move" onclick="addPointFromInputs('move')">Add Move Point</button>
      <button id="add-place" onclick="addPointFromInputs('place')">Add Place Point</button>
    </div>
    <div class="help">
      The three buttons add a point at the X and Y entered above.<br><br>
      <b>Left-click</b> the grid: pick point (open, descend, close, ascend).<br>
      <b>Middle-click</b>: move point.<br>
      <b>Right-click</b>: place point (descend, open, ascend, close).<br><br>
      The black dot and red/blue lines show the real robot position, reported by the ESP32.
    </div>
  </div>

</div>

<script>

const POINT_TYPES = {
  pick:  { prefix: "PK", label: "PICK POINT",  color: "#9c27b0", radius: 10, actions: ["open", "down", "close", "up"] },
  move:  { prefix: "M",  label: "MOVE POINT",  color: "#e53935", radius: 8,  actions: [] },
  place: { prefix: "PL", label: "PLACE POINT", color: "#ef6c00", radius: 10, actions: ["down", "open", "up", "close"] }
};

const ACTION_TEXT = { open: "Opening claw", close: "Closing claw", down: "Descending claw", up: "Ascending claw" };
const ACTION_URL  = { open: "/claw?state=open", close: "/claw?state=close", down: "/z?dir=down", up: "/z?dir=up" };
const BUTTON_TYPE = { 0: "pick", 1: "move", 2: "place" };

const grid = document.getElementById("grid");
const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");
const coordinates = document.getElementById("coordinates");
const cursorEl = document.getElementById("cursor");
const statusEl = document.getElementById("status");
const currentCourseName = document.getElementById("current-course-name");
const currentPoints = document.getElementById("current-points");
const courseList = document.getElementById("course-list");
const runCourseButton = document.getElementById("run-course");

/* Real robot position (reported by the ESP32) and mouse position */
let robotX = 225, robotY = 225, robotZ = 0;
let cursorX = null, cursorY = null;

let moving = false;
let courseID = 0;

let courses = [{ id: 1, name: "Course 1", points: [] }];
let selectedCourse = 0;

const sleep = function (ms) { return new Promise(function (r) { setTimeout(r, ms); }); };


/* ==================================
   ESP32 COMMUNICATION
================================== */

async function api(path) {
  const response = await fetch(path, { cache: "no-store" });
  if (!response.ok) throw new Error(path + " -> " + response.status);
  return response.json();
}

function applyStatus(s) {
  robotX = s.x;
  robotY = s.y;
  robotZ = s.z;
  updateCoordinates();
  draw();
}

function resetRunState() {
  moving = false;
  runCourseButton.disabled = false;
}

/*
 * Sends a command and waits until the ESP32 reports it finished.
 * Returns false if the course was stopped or the request failed.
 */
async function runCommand(path, token) {
  try {
    await api(path);

    while (true) {
      await sleep(60);

      if (token !== courseID) return false;

      const s = await api("/status");
      applyStatus(s);

      if (!s.busy) return true;
    }
  } catch (error) {
    statusEl.textContent = "ESP32 error: " + error.message;
    resetRunState();
    return false;
  }
}


/* ==================================
   CANVAS
================================== */

function resizeCanvas() {
  canvas.width = grid.clientWidth;
  canvas.height = grid.clientHeight;
  draw();
}

resizeCanvas();
window.addEventListener("resize", resizeCanvas);


/* ==================================
   MOUSE
================================== */

grid.addEventListener("mousemove", function (event) {
  const rect = grid.getBoundingClientRect();

  cursorX = event.clientX - rect.left - grid.clientLeft;
  cursorY = event.clientY - rect.top - grid.clientTop;

  cursorEl.textContent = `Cursor  X: ${Math.round(cursorX)}  Y: ${Math.round(cursorY)}`;
  draw();
});

grid.addEventListener("mouseleave", function () {
  cursorX = null;
  cursorY = null;
  cursorEl.textContent = "";
  draw();
});

/* Left = pick, middle (wheel) = move, right = place */
grid.addEventListener("mousedown", function (event) {
  const type = BUTTON_TYPE[event.button];
  if (!type) return;

  event.preventDefault();
  if (moving) return;

  const rect = grid.getBoundingClientRect();

  addPoint(
    type,
    Math.round(event.clientX - rect.left - grid.clientLeft),
    Math.round(event.clientY - rect.top - grid.clientTop)
  );
});

grid.addEventListener("contextmenu", function (event) { event.preventDefault(); });
grid.addEventListener("auxclick", function (event) { event.preventDefault(); });


/* ==================================
   POINTS
================================== */

function addPoint(type, x, y) {
  courses[selectedCourse].points.push({ x: x, y: y, type: type });

  updateCurrentCourse();
  updateCourseList();
  draw();

  statusEl.textContent = `${POINT_TYPES[type].label} created at X: ${x} Y: ${y}`;
}

function readInputs() {
  const x = Number(document.getElementById("input-x").value);
  const y = Number(document.getElementById("input-y").value);

  return {
    x: Math.round(Math.max(0, Math.min(canvas.width, x))),
    y: Math.round(Math.max(0, Math.min(canvas.height, y)))
  };
}

function addPointFromInputs(type) {
  if (moving) return;

  const p = readInputs();
  addPoint(type, p.x, p.y);
}

function updateCoordinates() {
  const z = Math.round(robotZ);

  coordinates.textContent =
    `X: ${Math.round(robotX)} | Y: ${Math.round(robotY)} | Z: ${z}%`;
}

function updateCurrentCourse() {
  const course = courses[selectedCourse];

  currentCourseName.textContent = course.name;
  currentPoints.innerHTML = "";

  if (course.points.length === 0) {
    currentPoints.innerHTML = `
      <div class="empty">
        No points saved.<br><br>
        Left-click = pick point<br>
        Middle-click = move point<br>
        Right-click = place point
      </div>`;
    return;
  }

  course.points.forEach(function (point, index) {
    const info = POINT_TYPES[point.type];
    const element = document.createElement("div");

    element.className = "point " + point.type;

    element.innerHTML = `
      <div class="point-title">${info.prefix}${index + 1}</div>
      <div class="point-type">${info.label}</div>
      <div class="point-coordinates">X: ${point.x} &nbsp;&nbsp; Y: ${point.y}</div>
      <button class="goto-button" onclick="goToPoint(${index})">Go To</button>
      <button class="delete-button" onclick="deletePoint(${index})">Delete</button>`;

    currentPoints.appendChild(element);
  });
}

function deletePoint(index) {
  if (moving) return;

  courses[selectedCourse].points.splice(index, 1);

  updateCurrentCourse();
  updateCourseList();
  draw();
}

function deleteAllPoints() {
  if (moving) return;

  const course = courses[selectedCourse];
  if (course.points.length === 0) return;

  course.points = [];

  updateCurrentCourse();
  updateCourseList();
  draw();

  statusEl.textContent = "All points deleted";
}


/* ==================================
   COURSES
================================== */

function updateCourseList() {
  courseList.innerHTML = "";

  courses.forEach(function (course, index) {
    const element = document.createElement("div");

    element.className = "course-item";
    if (index === selectedCourse) element.classList.add("selected");

    element.innerHTML = `
      <button class="course-select" onclick="selectCourse(${index})">
        ${course.name}
        <small>(${course.points.length} points)</small>
      </button>
      <button class="course-delete" onclick="deleteCourse(${index})">×</button>`;

    courseList.appendChild(element);
  });
}

function selectCourse(index) {
  if (moving) return;

  selectedCourse = index;

  updateCurrentCourse();
  updateCourseList();
  draw();

  statusEl.textContent = `${courses[index].name} selected`;
}

function createCourse() {
  if (moving) return;

  courses.push({ id: Date.now(), name: "Course " + (courses.length + 1), points: [] });
  selectedCourse = courses.length - 1;

  updateCurrentCourse();
  updateCourseList();
  draw();

  statusEl.textContent = `${courses[selectedCourse].name} created`;
}

function deleteCourse(index) {
  if (moving) return;

  if (courses.length === 1) {
    alert("You must have at least one course.");
    return;
  }

  courses.splice(index, 1);

  if (selectedCourse >= courses.length) selectedCourse = courses.length - 1;

  updateCurrentCourse();
  updateCourseList();
  draw();
}


/* ==================================
   MOVEMENT
================================== */

async function moveToPoint(x, y) {
  moving = true;

  statusEl.textContent = `Moving to X: ${x} Y: ${y}`;

  const done = await runCommand(`/move?x=${x}&y=${y}`, courseID);

  if (done) {
    moving = false;
    statusEl.textContent = `At X: ${x} Y: ${y}`;
  }
}

function moveToCoordinates() {
  if (moving) return;

  const p = readInputs();
  moveToPoint(p.x, p.y);
}

function goToPoint(index) {
  if (moving) return;

  const point = courses[selectedCourse].points[index];
  moveToPoint(point.x, point.y);
}


/* ==================================
   RUN / STOP COURSE
================================== */

async function runCourse() {
  if (moving) return;

  const course = courses[selectedCourse];

  if (course.points.length === 0) {
    alert("This course has no points.");
    return;
  }

  courseID++;
  const token = courseID;

  moving = true;
  runCourseButton.disabled = true;

  for (let i = 0; i < course.points.length; i++) {
    const point = course.points[i];
    const info = POINT_TYPES[point.type];
    const name = info.prefix + (i + 1);

    statusEl.textContent = `Moving to ${name} | X: ${point.x} Y: ${point.y}`;

    if (!(await runCommand(`/move?x=${point.x}&y=${point.y}`, token))) return;

    for (const action of info.actions) {
      statusEl.textContent = `${name}: ${ACTION_TEXT[action]}...`;

      if (!(await runCommand(ACTION_URL[action], token))) return;
    }
  }

  resetRunState();
  statusEl.textContent = `${course.name} finished`;
}

async function stopCourse() {
  courseID++;

  try {
    await api("/stop");

    /* Wait for the motors to halt, then read the real position */
    for (let i = 0; i < 50; i++) {
      const s = await api("/status");
      applyStatus(s);

      if (!s.busy) break;

      await sleep(60);
    }
  } catch (error) {
    statusEl.textContent = "ESP32 error: " + error.message;
  }

  resetRunState();
  statusEl.textContent = `Stopped at X: ${Math.round(robotX)} Y: ${Math.round(robotY)}`;
}


/* ==================================
   DRAW
================================== */

function draw() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const course = courses[selectedCourse];

  if (course) {
    course.points.forEach(function (point, index) {
      const info = POINT_TYPES[point.type];

      ctx.beginPath();
      ctx.arc(point.x, point.y, info.radius, 0, Math.PI * 2);
      ctx.fillStyle = info.color;
      ctx.fill();

      if (info.actions.length > 0) {
        ctx.fillStyle = "white";
        ctx.font = "bold 10px Arial";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(info.prefix, point.x, point.y);
      }

      ctx.textAlign = "start";
      ctx.textBaseline = "alphabetic";
      ctx.fillStyle = "black";
      ctx.font = "bold 14px Arial";
      ctx.fillText(info.prefix + (index + 1), point.x + 12, point.y - 10);
    });
  }

  /* Mouse cursor marker (where a click will create a point) */
  if (cursorX !== null) {
    ctx.beginPath();
    ctx.arc(cursorX, cursorY, 6, 0, Math.PI * 2);
    ctx.strokeStyle = "#999";
    ctx.lineWidth = 1;
    ctx.stroke();
  }

  /* Real robot position */
  ctx.beginPath();
  ctx.moveTo(robotX, 0);
  ctx.lineTo(robotX, canvas.height);
  ctx.strokeStyle = "rgba(255, 0, 0, 0.7)";
  ctx.lineWidth = 2;
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(0, robotY);
  ctx.lineTo(canvas.width, robotY);
  ctx.strokeStyle = "rgba(0, 100, 255, 0.7)";
  ctx.lineWidth = 2;
  ctx.stroke();

  ctx.beginPath();
  ctx.arc(robotX, robotY, 5, 0, Math.PI * 2);
  ctx.fillStyle = "black";
  ctx.fill();
}


/* ==================================
   INITIALIZE
================================== */

updateCurrentCourse();
updateCourseList();
draw();

api("/status").then(applyStatus).catch(function () {
  statusEl.textContent = "Cannot reach the ESP32";
});

</script>
</body>
</html>
)HTML";


/* ==================================
   MOTOR CONTROL (runs in its own task)
================================== */

// Moves X and Y together so diagonal moves are straight lines.
void stepXY(long dx, long dy) {
  digitalWrite(X_DIR, ((dx >= 0) ^ X_INVERT) ? HIGH : LOW);
  digitalWrite(Y_DIR, ((dy >= 0) ^ Y_INVERT) ? HIGH : LOW);
  delayMicroseconds(10);

  long ax = labs(dx);
  long ay = labs(dy);
  long n = max(ax, ay);
  long ex = 0, ey = 0;

  for (long i = 0; i < n && !abortFlag; i++) {
    ex += ax;
    ey += ay;

    bool stepX = ex >= n;
    bool stepY = ey >= n;

    if (stepX) ex -= n;
    if (stepY) ey -= n;

    if (stepX) digitalWrite(X_STEP, HIGH);
    if (stepY) digitalWrite(Y_STEP, HIGH);
    delayMicroseconds(STEP_US);

    digitalWrite(X_STEP, LOW);
    digitalWrite(Y_STEP, LOW);
    delayMicroseconds(STEP_US);

    if (stepX) posX += (dx > 0) ? 1 : -1;
    if (stepY) posY += (dy > 0) ? 1 : -1;

    if (i % 100 == 99) vTaskDelay(1);   // let the other tasks run
  }
}

void moveZ(long target) {
  long dz = target - posZ;

  digitalWrite(Z_DIR, ((dz >= 0) ^ Z_INVERT) ? HIGH : LOW);
  delayMicroseconds(10);

  for (long i = 0; i < labs(dz) && !abortFlag; i++) {
    digitalWrite(Z_STEP, HIGH);
    delayMicroseconds(STEP_US);
    digitalWrite(Z_STEP, LOW);
    delayMicroseconds(STEP_US);

    posZ += (dz > 0) ? 1 : -1;

    if (i % 100 == 99) vTaskDelay(1);
  }
}

// The claw is a servo: it moves to the open or closed angle, then waits for it to finish.
void clawOpen() {
  Serial.println("Claw: open");
  clawServo.write(CLAW_OPEN_ANGLE);
  vTaskDelay(pdMS_TO_TICKS(CLAW_MS));
}

void clawClose() {
  Serial.println("Claw: close");
  clawServo.write(CLAW_CLOSED_ANGLE);
  vTaskDelay(pdMS_TO_TICKS(CLAW_MS));
}

void motionTask(void* parameter) {
  Cmd c;

  for (;;) {
    if (xQueueReceive(cmdQueue, &c, portMAX_DELAY) == pdTRUE) {
      switch (c.type) {
        case CMD_MOVE:
          stepXY(c.a - posX, c.b - posY);
          break;
        case CMD_Z:
          moveZ(c.a);
          break;
        case CMD_CLAW_OPEN:
          clawOpen();
          break;
        case CMD_CLAW_CLOSE:
          clawClose();
          break;
      }

      busy = false;
    }
  }
}


/* ==================================
   WEB HANDLERS
================================== */

bool startCommand(CmdType type, long a, long b) {
  if (busy) {
    server.send(409, "text/plain", "Busy");
    return false;
  }

  busy = true;
  abortFlag = false;

  Cmd c = { type, a, b };
  xQueueSend(cmdQueue, &c, 0);

  server.send(200, "application/json", "{\"ok\":true}");
  return true;
}

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  String json = "{\"x\":" + String(posX / (float)STEPS_PER_PX, 1) +
                ",\"y\":" + String(posY / (float)STEPS_PER_PX, 1) +
                ",\"z\":" + String(posZ * 100 / Z_TRAVEL_STEPS) +
                ",\"busy\":" + (busy ? "true" : "false") + "}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleMove() {
  long x = constrain(server.arg("x").toInt(), 0, MAX_PX);
  long y = constrain(server.arg("y").toInt(), 0, MAX_PX);

  startCommand(CMD_MOVE, x * STEPS_PER_PX, y * STEPS_PER_PX);
}

void handleZ() {
  bool down = server.arg("dir") == "down";

  startCommand(CMD_Z, down ? Z_TRAVEL_STEPS : 0, 0);
}

void handleClaw() {
  bool open = server.arg("state") == "open";

  startCommand(open ? CMD_CLAW_OPEN : CMD_CLAW_CLOSE, 0, 0);
}

void handleStop() {
  abortFlag = true;

  server.send(200, "application/json", "{\"ok\":true}");
}


/* ==================================
   SETUP / LOOP
================================== */

void setup() {
  Serial.begin(115200);

  pinMode(X_STEP, OUTPUT);
  pinMode(X_DIR, OUTPUT);
  pinMode(Y_STEP, OUTPUT);
  pinMode(Y_DIR, OUTPUT);
  pinMode(Z_STEP, OUTPUT);
  pinMode(Z_DIR, OUTPUT);

  clawServo.setPeriodHertz(50);
  clawServo.attach(SERVO_PIN, 500, 2400);
  clawServo.write(CLAW_CLOSED_ANGLE);

  cmdQueue = xQueueCreate(4, sizeof(Cmd));
  xTaskCreatePinnedToCore(motionTask, "motion", 4096, NULL, 1, NULL, 1);

  WiFi.mode(WIFI_AP_STA);

  // Own network: join it from a phone or PC, then open http://192.168.4.1
  WiFi.softAP(apName, apPassword);

  Serial.print("Access point \"");
  Serial.print(apName);
  Serial.print("\" ready, IP: ");
  Serial.println(WiFi.softAPIP());

  // Also try to join your router (2.4 GHz only)
  WiFi.begin(ssid, password);

  Serial.print("Connecting to router");

  for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Router connected, IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Router not connected - use the access point instead.");
  }

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/move", handleMove);
  server.on("/z", handleZ);
  server.on("/claw", handleClaw);
  server.on("/stop", handleStop);

  server.begin();

  Serial.println("Web server started!");
}

void loop() {
  server.handleClient();
  delay(2);
}
