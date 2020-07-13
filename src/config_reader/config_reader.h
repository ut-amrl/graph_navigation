// Copyright 2019 Kyle Vedder (kvedder@seas.upenn.edu), 2018 Ishan Khatri
// (ikhatri@umass.edu)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// ========================================================================
#ifndef CONFIGREADER_CONFIG_READER_H_
#define CONFIGREADER_CONFIG_READER_H_

extern "C" {
#include <libgen.h>
#include <sys/epoll.h>
#include <sys/inotify.h>
#include <unistd.h>
}

#include <algorithm>
#include <atomic>
#include <fstream>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "config_reader/lua_script.h"
#include "config_reader/macros.h"
#include "config_reader/types/config_generic.h"
#include "config_reader/types/config_numeric.h"
#include "config_reader/types/type_interface.h"

namespace config_reader {

inline void LuaRead(const std::vector<std::string>& files) {
  // Create the LuaScript object
  LuaScript script(files);
  // Loop through the unordered map
  for (const auto& pair : MapSingleton::Singleton()) {
    config_types::TypeInterface* t = pair.second.get();
    if (t->GetType() == config_types::CNULL) {
      std::cerr << "Key has a type CNULL!" << std::endl;
      return;
    }
    t->SetValue(&script);
  }
  *MapSingleton::NewKeyAdded() = false;
}

class ConfigReader {
  std::atomic_bool is_running_;
  std::thread daemon_;

  void InitDaemon(const std::vector<std::string> files) {
    static constexpr int kEventSize = sizeof(inotify_event);
    static constexpr int kEventBufferLength = (1024 * (kEventSize + 16));
    static constexpr int kInotifySleep = 50;
    std::array<char, kEventBufferLength> buffer;

    // Initialize inotify
    int fd = inotify_init();
    if (fd < 0) {
      std::cerr << "ERROR: Couldn't initialize inotify" << std::endl;
      exit(-1);
    }

    // Add a listener on each parent directory
    for (const std::string& file : files) {
      int wd = inotify_add_watch(fd, file.c_str(), IN_MODIFY);

      if (wd < 0) {
        std::cerr << "ERROR: Couldn't add watch to the file: "
                  << file
                  << std::endl;
        perror("Reason");
        return;
      }
    }

    int epfd = epoll_create(1);
    if (epfd < 0) {
      std::cerr << "ERROR: Call to epoll_create failed." << std::endl;
    }

    epoll_event ready_to_read = {};
    ready_to_read.data.fd = fd;
    ready_to_read.events = EPOLLIN;
    if (epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ready_to_read)) {
      std::cerr << "ERROR: Call to epoll_ctl failed." << std::endl;
    }

    epoll_event epoll_events;

    auto last_notify = std::chrono::system_clock::now();
    bool needs_update = false;

    // Loop forever, checking for changes to the files above
    while (is_running_) {
      // Wait for 50 ms for there to be an available inotify event
      int nr_events = epoll_wait(epfd, &epoll_events, 1, kInotifySleep);

      // Handle addition of keys after Daemon starts.
      if (*MapSingleton::NewKeyAdded()) {
        LuaRead(files);
        needs_update = false;
        continue;
      }

      if (nr_events > 0) {
        // Else if the inotify fd has recieved something that can be read
        int length = read(fd, &buffer, kEventBufferLength);
        if (length < 0 || length > kEventBufferLength) {
          std::cerr << "ERROR: Inotify read failed" << std::endl;
          continue;
        }

        for (int i = 0; i < length;) {
          inotify_event* event = reinterpret_cast<inotify_event*>(&buffer[i]);
          i += kEventSize + event->len;

          last_notify = std::chrono::system_clock::now();
          needs_update = true;
        }
      }
      if (needs_update && std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::system_clock::now() - last_notify)
                                  .count() > 2 * kInotifySleep) {
        LuaRead(files);
        needs_update = false;
      }
    }

    // Clean up
    close(epfd);
    close(fd);
  }
  void CreateDaemon(const std::vector<std::string> files) {
    LuaRead(files);
    is_running_ = true;
    daemon_ = std::thread(&ConfigReader::InitDaemon, this, files);
  }

  void Stop() {
    is_running_ = false;
    if (daemon_.joinable()) {
      daemon_.join();
    }
  }

 public:
  ConfigReader() = delete;
  ConfigReader(const std::vector<std::string>& files) { CreateDaemon(files); }
  ~ConfigReader() { Stop(); }
};

}  // namespace config_reader

#endif  // CONFIGREADER_CONFIG_READER_H_
