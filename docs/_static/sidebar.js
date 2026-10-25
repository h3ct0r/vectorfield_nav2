// Copyright 2026 Héctor Azpúrua
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

// Keeps the left sidebar where the reader left it. Furo reloads the whole page on every link, so
// its scrolling navigation (.sidebar-scroll) starts at the top again; this saves its position
// when a page is left and restores it on the next one. Without a saved position (a first visit,
// or a page reached from elsewhere), it brings the current page's entry into view instead.
(function () {
  'use strict';

  var KEY = 'vf-sidebar-scroll';

  function read() {
    try {
      var value = window.sessionStorage.getItem(KEY);
      return value === null ? null : Number(value);
    } catch (e) {
      return null; // storage blocked: nothing to restore
    }
  }

  function save(scroller) {
    try {
      window.sessionStorage.setItem(KEY, String(scroller.scrollTop));
    } catch (e) {
      // storage blocked: the sidebar starts at the top, as without this script
    }
  }

  function setup() {
    var scroller = document.querySelector('.sidebar-scroll');
    if (!scroller) {
      return;
    }
    // Furo scrolls the sidebar smoothly; the restore must be instant.
    var behavior = scroller.style.scrollBehavior;
    scroller.style.scrollBehavior = 'auto';
    var saved = read();
    if (saved !== null && isFinite(saved)) {
      scroller.scrollTop = saved;
    }
    var current = scroller.querySelector('.current-page > a');
    if (current) {
      var box = scroller.getBoundingClientRect();
      var link = current.getBoundingClientRect();
      if (link.top < box.top || link.bottom > box.bottom) {
        // Not in view (no saved position, or the previous/next buttons moved far): centre it,
        // scrolling the sidebar only (scrollIntoView would also scroll the page).
        scroller.scrollTop += link.top - box.top - (box.height - link.height) / 2;
      }
    }
    scroller.style.scrollBehavior = behavior;
    window.addEventListener('pagehide', function () {
      save(scroller);
    });
    scroller.addEventListener('click', function () {
      save(scroller);
    });
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', setup);
  } else {
    setup();
  }
})();
