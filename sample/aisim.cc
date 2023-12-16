// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdio>
#include <cstring>
#include <vector>
#include <chrono>
#include <thread>

#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#define PORT 8080

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include "array_safety.h"


// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d);
  }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

std::vector<mjtNum> CtrlNoise(const mjModel* m) {
  static int step=0;
  std::vector<mjtNum> ctrl;

  // inject pseudo-random control noise
  // create pseudo-random control sequence
  mjtNum ctrlnoise = 0.01;

  for (int i = 0; i < m->nu; i++) {
    mjtNum center = 0.0;
    mjtNum radius = 1.0;
    mjtNum* range = m->actuator_ctrlrange + 2 * i;
    if (m->actuator_ctrllimited[i]) {
      center = (range[1] + range[0]) / 2;
      radius = (range[1] - range[0]) / 2;
    }
    radius *= ctrlnoise;
    step++;
    ctrl.push_back(center + radius * (2 * mju_Halton(step, i+2) - 1));
  }
  return ctrl;
}

void send_controls(const char* actuator_name, mjtNum actuator_val) {
    for (int i = 0; i < m->nu; i++) {
      if (!std::strcmp(actuator_name, m->names + m->name_actuatoradr[i])) {
          mju_copy(&d->ctrl[i], &actuator_val, 1);
          break;
      }
    }
}

struct CommunicateParams {
  int terminate;
  char buf[1000];
};

void communicate(CommunicateParams* communicate_params)
{
  int server_fd = -1;
  int new_socket = -1;
  ssize_t valread;
  struct sockaddr_in address;
  int opt = 1;
  socklen_t addrlen = sizeof(address);
  char buffer[1024] = { 0 };
  char hello[] = "Hello from server";

  while(!communicate_params->terminate) {

    if (server_fd < 0) {
      // Creating socket file descriptor
      if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("socket failed");
        return;
      }

      // Forcefully attaching socket to the port 8080
      if (setsockopt(server_fd, SOL_SOCKET,
            SO_REUSEADDR | SO_REUSEPORT, &opt,
            sizeof(opt))) {
        perror("setsockopt");
        return;
      }
      address.sin_family = AF_INET;
      address.sin_addr.s_addr = INADDR_ANY;
      address.sin_port = htons(PORT);

      // Forcefully attaching socket to the port 8080
      if (bind(server_fd, (struct sockaddr*)&address,
          sizeof(address))
        < 0) {
        perror("bind failed");
        return;
      }

      if (listen(server_fd, 3) < 0) {
        perror("listen");
        return;
      }
    }

    if (new_socket < 0) {
      new_socket = accept(server_fd, (struct sockaddr*)&address, &addrlen);
      if (new_socket < 0) {
        perror("accept");
        return;
      }
    }

    if (new_socket >= 0) {    
      valread = read(new_socket, buffer,
            1024 - 1); // subtract 1 for the null
                  // terminator at the end
      if (valread > 0) {
        printf("%s\n", buffer);
        send(new_socket, hello, strlen(hello), 0);
      }
    }
  }

  // closing the connected socket
  close(new_socket);
  // closing the listening socket
  close(server_fd);
}

/*
command line examples:  
gdb --args ./aisim ../model/humanoid.xml hip_x_right 0.1
./aisim ../model/humanoid.xml elbow_right 0.05 
*/

// main function
int main(int argc, const char** argv) {

  char actuator_name[100] = {0};  // see actuator in humanoid.xml, e.g. elbow_right, hip_x_right, etc.
  double actuator_val;

  // check command-line arguments
  if (argc < 2) {
    std::printf(" USAGE:  aisim modelfile [actuator_name actuator_val]\n");
    return 0;
  }

  if (argc > 2) {
    if (std::strlen(argv[2]) > sizeof(actuator_name)/sizeof(char)) {
      printf("actuator_name too long\n");
      return 1;
    }

    std::sscanf(argv[2], "%s", actuator_name);  
    std::sscanf(argv[3], "%lf", &actuator_val);  
  }

  // load and compile model
  char error[1000] = "Could not load binary model";
  if (std::strlen(argv[1])>4 && !std::strcmp(argv[1]+std::strlen(argv[1])-4, ".mjb")) {
    m = mj_loadModel(argv[1], 0);
  } else {
    m = mj_loadXML(argv[1], 0, error, 1000);
  }
  if (!m) {
    mju_error("Load model error: %s", error);
  }

  // make data
  d = mj_makeData(m);

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  CommunicateParams communicate_params;
  std::memset(&communicate_params, 0, sizeof(communicate_params));
  std::thread communication_thread = std::thread(communicate, &communicate_params);  

  using namespace std::chrono;
  auto prev_time = system_clock::now();

  // run main loop, target real-time simulation and 60 fps rendering
  while (!glfwWindowShouldClose(window)) {

    if (duration_cast<duration<double>>(system_clock::now() - prev_time).count() > 1.0/20.0) {
      if (std::strlen(actuator_name) > 0) {
        send_controls(actuator_name, actuator_val);
      }
      mj_step(m, d);    
      prev_time = system_clock::now();
      //actuator_val=actuator_val+0.01;
    }

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  communicate_params.terminate = 1;
  communication_thread.join();  


  //free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif

  return 1;
}
