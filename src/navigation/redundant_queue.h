// Copyright (c) 2018 joydeepb@cs.umass.edu
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <algorithm>
#include <queue>
#include <unordered_set>
#include <utility>

#ifndef REDUNDANT_QUEUE
#define REDUNDANT_QUEUE

using std::priority_queue;
using std::unordered_set;
using std::pair;
using std::make_pair;

template<class Value, class Priority, class Hash>
class RedundantQueue {
  struct Comparer {
    bool operator()(const pair<Value, Priority>& v1,
                    const pair<Value, Priority>& v2) {
      return (v1.second < v2.second);
    }
  };

 public:
  explicit RedundantQueue(const Hash& h) :
    values_(comparer_),
    popped_(100, h),
    queue_entries_(100, h) {}
  // Insert a new value, with the specified priority. If the value
  // already exists, its priority is updated.
  void Push(const Value& v, const Priority& p) {
    values_.emplace(make_pair(v, p));
    queue_entries_.emplace(v);
  }

  // Retreive the value with the highest priority.
  Value Pop() {
    Value v = values_.top().first;
    queue_entries_.erase(v);
    values_.pop();
    popped_.emplace(v);
    return v;
  }

  // Returns true iff the priority queue is empty.
  bool Empty() const {
    for (const auto& x : queue_entries_) {
      if (popped_.count(x) == 0) return false;
    }
    return true;
  }

  // Returns true iff the specified value exists.
  bool Exists(const Value& v) const {
    for (const auto& x : queue_entries_) {
      if (x == v && popped_.count(v) == 0) return true;
    }
    return false;
  }

 private:
  Comparer comparer_;
  priority_queue<pair<Value, Priority>,
      std::vector<pair<Value, Priority>>,
      Comparer> values_;
  unordered_set<Value, Hash> popped_;
  unordered_set<Value, Hash> queue_entries_;
};

#endif  // REDUNDANT_QUEUE
