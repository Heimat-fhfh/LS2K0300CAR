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
    `track_kind: ${stat.track_kind}  servo_angle: ${stat.servo_angle}  motor_speed: ${stat.motor_speed}\n` +
    `inflection(L/R): ${stat.inflection_left}/${stat.inflection_right}   bend(L/R): ${stat.bend_left}/${stat.bend_right}`;
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
