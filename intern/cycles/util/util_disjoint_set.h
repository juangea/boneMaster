/*
 * Copyright 2011-2013 Blender Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __UTIL_DISJOINT_SET_H__
#define __UTIL_DISJOINT_SET_H__

#include <utility>
#include "util_array.h"

CCL_NAMESPACE_BEGIN

class DisjointSet {
 private:
  array<size_t> parents;
  array<size_t> ranks;

 public:
  DisjointSet(size_t size) : parents(size), ranks(size)
  {
    for (size_t i = 0; i < size; i++) {
      parents[i] = i;
      ranks[i] = 0;
    }
  }

  size_t find(size_t x)
  {
    if (parents[x] != x) {
      parents[x] = find(parents[x]);
    }
    return parents[x];
  }

  void join(size_t x, size_t y)
  {
    size_t xRoot = find(x);
    size_t yRoot = find(y);

    if (xRoot == yRoot) {
      return;
    }

    if (ranks[xRoot] < ranks[yRoot]) {
      std::swap(xRoot, yRoot);
    }
    parents[yRoot] = xRoot;

    if (ranks[xRoot] == ranks[yRoot]) {
      ranks[xRoot]++;
    }
  }
};

CCL_NAMESPACE_END

#endif /* __UTIL_DISJOINT_SET_H__ */
