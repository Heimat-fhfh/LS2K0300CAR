let frameCount = 0;
let timer = null;

const slider = document.getElementById('idx');
const metaEl = document.getElementById('meta');
const datasetInfo = document.getElementById('datasetInfo');
const playBtn = document.getElementById('playBtn');

document.getElementById('prevBtn').addEventListener('click', prevFrame);
document.getElementById('nextBtn').addEventListener('click', nextFrame);
playBtn.addEventListener('click', togglePlay);
slider.addEventListener('input', () => render(Number(slider.value)));

const ROAD_NAMES = {
  0: 'Normol', 1: 'Straight', 2: 'Cross', 3: 'Ramp',
  4: 'L-Cirque', 5: 'R-Cirque', 6: 'Forkin', 7: 'Forkout',
  8: 'Barn_out', 9: 'Barn_in', 10: 'CrossTure'
};

const TRACK_NAMES = {
  0: 'Straight', 1: 'Bend',
  2: 'L-Across', 3: 'R-Across',
  4: 'L-Circle', 5: 'R-Circle'
};

function roadName(t) {
  return ROAD_NAMES[t] || ('?' + t);
}

function trackName(t) {
  return TRACK_NAMES[t] || ('?' + t);
}

async function loadMeta() {
  const res = await fetch('/api/meta');
  const meta = await res.json();
  frameCount = meta.frame_count;
  datasetInfo.textContent = `数据源: ${meta.dataset}`;
  slider.max = Math.max(0, frameCount - 1);
  slider.value = 0;
  await render(0);
}

async function render(idx) {
  const ts = Date.now();
  document.getElementById('img_orig').src = `/api/image?idx=${idx}&type=orig&t=${ts}`;
  document.getElementById('img_gray').src = `/api/image?idx=${idx}&type=gray&t=${ts}`;
  document.getElementById('img_otsu').src = `/api/image?idx=${idx}&type=otsu&t=${ts}`;
  document.getElementById('img_track').src = `/api/image?idx=${idx}&type=track&t=${ts}`;
  document.getElementById('img_all').src = `/api/image?idx=${idx}&type=all&t=${ts}`;

  const statRes = await fetch(`/api/frame?idx=${idx}`);
  const stat = await statRes.json();
  metaEl.textContent =
    `Frame: ${idx + 1}/${frameCount}\n` +
    `file: ${stat.file}\n` +
    `TK:${trackName(stat.track_kind)}  Road:${roadName(stat.my_zf_road_type)}  Det:${stat.my_zf_det_true}\n` +
    `Ring:${stat.my_zf_rings} Flag:${stat.my_zf_ring_flag} Size:${stat.my_zf_ring_size}\n` +
    `OFFLine:${stat.my_zf_off_line}  WL:${stat.my_zf_white_line}\n` +
    `SErrPx:${stat.steer_error_px}  TBS:${(stat.target_base_speed || 0).toFixed(1)}`;
}

function prevFrame() {
  const v = Math.max(0, Number(slider.value) - 1);
  slider.value = v;
  render(v);
}

function nextFrame() {
  const v = Math.min(Math.max(frameCount - 1, 0), Number(slider.value) + 1);
  slider.value = v;
  render(v);
}

function togglePlay() {
  if (timer) {
    clearInterval(timer);
    timer = null;
    playBtn.textContent = '播放';
    return;
  }

  playBtn.textContent = '暂停';
  timer = setInterval(() => {
    let v = Number(slider.value) + 1;
    if (v >= frameCount) {
      v = 0;
    }
    slider.value = v;
    render(v);
  }, 150);
}

loadMeta();
