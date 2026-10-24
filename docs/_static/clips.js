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

// The gallery clips (docs/vf_clips.py) autoplay, muted and looped. This plays each one only
// while it is on screen, and none for readers who prefer reduced motion: their clips stop on
// the poster frame and get controls instead.
(function () {
  'use strict';

  function setup() {
    var clips = Array.prototype.slice.call(document.querySelectorAll('video.vf-clip'));
    if (clips.length === 0) {
      return;
    }
    var reduce =
      window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    if (reduce) {
      clips.forEach(function (clip) {
        clip.removeAttribute('autoplay');
        clip.pause();
        clip.controls = true;
      });
      return;
    }
    if (!('IntersectionObserver' in window)) {
      return;
    }
    var observer = new IntersectionObserver(
      function (entries) {
        entries.forEach(function (entry) {
          var clip = entry.target;
          if (entry.isIntersecting) {
            var playing = clip.play();
            if (playing && playing.catch) {
              playing.catch(function () {}); // autoplay refused: the poster stays
            }
          } else {
            clip.pause();
          }
        });
      },
      { threshold: 0.2 }
    );
    clips.forEach(function (clip) {
      observer.observe(clip);
    });
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', setup);
  } else {
    setup();
  }
})();
