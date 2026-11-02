<script setup>
// 导航地图：2D 栅格 + 建图轨迹 + 全局路径 + 实时障碍点 + 车辆 + 目标
// mode 为 'goal' / 'reloc' 时：按下确定位置，拖动确定朝向，松开后 emit('pick', {x, y, yaw})
import { onMounted, onUnmounted, ref, watch } from 'vue'

const props = defineProps({
  map: Object,            // meta.json（grid.origin / resolution / width / height）
  gridUrl: String,
  traj: Array,            // [[x, y], ...]
  state: Object,          // nav_bridge 推送的 state
  mode: { type: String, default: 'view' },
})
const emit = defineEmits(['pick'])

const canvasEl = ref(null)
const wrapEl = ref(null)
const follow = ref(false)

const view = { vx: 0, vy: 0, scale: 20 }
let W = 0, H = 0, dpr = 1
let gridImg = null
let needDraw = true
let raf = 0
let drag = null              // 点选中：{x, y, sx, sy, yaw}
const pointers = new Map()
let pinchDist = 0

const decodeScan = (b64) => {
  if (!b64) return new Int16Array(0)
  const bin = atob(b64)
  const bytes = new Uint8Array(bin.length)
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i)
  return new Int16Array(bytes.buffer)
}

function loadGrid() {
  gridImg = null
  if (!props.gridUrl) return
  const img = new Image()
  img.onload = () => { gridImg = img; needDraw = true }
  img.src = props.gridUrl
}

function fitMap() {
  const m = props.map
  if (!m || !W) return
  const gw = m.grid.width * m.grid.resolution, gh = m.grid.height * m.grid.resolution
  view.vx = m.grid.origin[0] + gw / 2
  view.vy = m.grid.origin[1] + gh / 2
  view.scale = Math.min(W / gw, H / gh) * 0.95
  follow.value = false
  needDraw = true
}

watch(() => props.gridUrl, () => { loadGrid(); fitMap() })
watch(() => props.state, () => (needDraw = true))
watch(() => props.traj, () => (needDraw = true))
watch(follow, () => (needDraw = true))

const toScreen = (x, y) => [W / 2 + (x - view.vx) * view.scale, H / 2 - (y - view.vy) * view.scale]
const toWorld = (sx, sy) => [view.vx + (sx - W / 2) / view.scale, view.vy - (sy - H / 2) / view.scale]

function arrow(ctx, x, y, yaw, size, fill, stroke = '#000') {
  const [sx, sy] = toScreen(x, y)
  ctx.save()
  ctx.translate(sx, sy)
  ctx.rotate(-yaw)
  ctx.fillStyle = fill
  ctx.strokeStyle = stroke
  ctx.lineWidth = 1.5
  ctx.beginPath()
  ctx.moveTo(size, 0); ctx.lineTo(-size * 0.65, size * 0.6); ctx.lineTo(-size * 0.3, 0); ctx.lineTo(-size * 0.65, -size * 0.6)
  ctx.closePath()
  ctx.fill(); ctx.stroke()
  ctx.restore()
}

function polyline(ctx, pts, color, width, dash = []) {
  if (!pts || pts.length < 2) return
  ctx.strokeStyle = color
  ctx.lineWidth = width
  ctx.setLineDash(dash)
  ctx.beginPath()
  pts.forEach(([x, y], i) => {
    const [sx, sy] = toScreen(x, y)
    i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy)
  })
  ctx.stroke()
  ctx.setLineDash([])
}

function draw() {
  const ctx = canvasEl.value.getContext('2d')
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
  ctx.fillStyle = '#c9ccd1'
  ctx.fillRect(0, 0, W, H)
  const st = props.state || {}
  if (follow.value && st.pose) { view.vx = st.pose[0]; view.vy = st.pose[1] }

  const m = props.map
  if (m && gridImg) {
    const res = m.grid.resolution
    const [sx, sy] = toScreen(m.grid.origin[0], m.grid.origin[1] + m.grid.height * res)
    ctx.imageSmoothingEnabled = view.scale * res < 2
    ctx.drawImage(gridImg, sx, sy, m.grid.width * res * view.scale, m.grid.height * res * view.scale)
  }

  polyline(ctx, props.traj, 'rgba(120,120,200,0.55)', 2, [6, 5])
  polyline(ctx, st.plan, '#16a34a', 3)

  // 实时障碍点（地面以上 0.1~1.6 m）
  const scan = decodeScan(st.scan)
  if (scan.length) {
    const u = st.scan_unit || 0.02
    ctx.fillStyle = '#e11d48'
    for (let i = 0; i < scan.length; i += 2) {
      const [sx, sy] = toScreen(scan[i] * u, scan[i + 1] * u)
      ctx.fillRect(sx - 1.5, sy - 1.5, 3, 3)
    }
  }

  if (st.goal && ['pending', 'active'].includes(st.goal_state)) arrow(ctx, ...st.goal, 13, '#f59e0b')

  if (st.pose) {
    // 车身轮廓：pose 为雷达位置，footprint = [前, 后, 半宽]（相对雷达，见 config/nav/costmap_common.yaml）
    const [x, y, yaw] = st.pose
    const [front, rear, hw] = st.footprint || [0.16, -0.68, 0.28]
    const c = Math.cos(yaw), s = Math.sin(yaw)
    ctx.fillStyle = 'rgba(37,99,235,0.25)'
    ctx.strokeStyle = '#1d4ed8'
    ctx.lineWidth = 1.5
    ctx.beginPath()
    ;[[front, hw], [front, -hw], [rear, -hw], [rear, hw]].forEach(([dx, dy], i) => {
      const [sx, sy] = toScreen(x + c * dx - s * dy, y + s * dx + c * dy)
      i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy)
    })
    ctx.closePath(); ctx.fill(); ctx.stroke()
    arrow(ctx, x, y, yaw, 12, '#2563eb', '#fff')
  }

  if (drag) arrow(ctx, drag.x, drag.y, drag.yaw, 15, props.mode === 'goal' ? '#f59e0b' : '#a855f7', '#fff')

  // 比例尺
  const m1 = 100 / view.scale, p = Math.pow(10, Math.floor(Math.log10(m1)))
  const bar = [1, 2, 5, 10].map(k => k * p).find(v => v >= m1)
  ctx.strokeStyle = ctx.fillStyle = '#111'
  ctx.lineWidth = 2
  ctx.beginPath()
  ctx.moveTo(16, H - 16); ctx.lineTo(16 + bar * view.scale, H - 16)
  ctx.stroke()
  ctx.font = '12px sans-serif'
  ctx.fillText(bar >= 1 ? `${bar} m` : `${bar * 100} cm`, 16, H - 22)
}

function loop() {
  if (needDraw && canvasEl.value && W) { draw(); needDraw = false }
  raf = requestAnimationFrame(loop)
}

function resize() {
  const r = wrapEl.value.getBoundingClientRect()
  const first = !W
  dpr = window.devicePixelRatio || 1
  W = r.width; H = r.height
  canvasEl.value.width = Math.round(W * dpr)
  canvasEl.value.height = Math.round(H * dpr)
  if (first) fitMap()
  needDraw = true
}

// ------------------------------------------------------------------ 交互
function local(e) {
  const r = canvasEl.value.getBoundingClientRect()
  return [e.clientX - r.left, e.clientY - r.top]
}

function zoomAt(f, sx, sy) {
  const [wx, wy] = toWorld(sx, sy)
  view.scale = Math.min(400, Math.max(2, view.scale * f))
  if (!follow.value) {
    view.vx = wx - (sx - W / 2) / view.scale
    view.vy = wy + (sy - H / 2) / view.scale
  }
  needDraw = true
}

function onWheel(e) {
  e.preventDefault()
  zoomAt(Math.exp(-e.deltaY * 0.0015), ...local(e))
}

function onDown(e) {
  canvasEl.value.setPointerCapture(e.pointerId)
  if (props.mode !== 'view' && pointers.size === 0) {
    const [sx, sy] = local(e)
    const [x, y] = toWorld(sx, sy)
    drag = { x, y, sx, sy, yaw: props.state?.pose ? props.state.pose[2] : 0 }
    needDraw = true
    return
  }
  pointers.set(e.pointerId, [e.clientX, e.clientY])
  if (pointers.size === 2) {
    const [a, b] = [...pointers.values()]
    pinchDist = Math.hypot(a[0] - b[0], a[1] - b[1])
  }
}

function onMove(e) {
  if (drag) {
    const [sx, sy] = local(e)
    if (Math.hypot(sx - drag.sx, sy - drag.sy) > 8) drag.yaw = Math.atan2(-(sy - drag.sy), sx - drag.sx)
    needDraw = true
    return
  }
  const prev = pointers.get(e.pointerId)
  if (!prev) return
  pointers.set(e.pointerId, [e.clientX, e.clientY])
  if (pointers.size === 1) {
    follow.value = false
    view.vx -= (e.clientX - prev[0]) / view.scale
    view.vy += (e.clientY - prev[1]) / view.scale
    needDraw = true
  } else if (pointers.size === 2) {
    const [a, b] = [...pointers.values()]
    const d = Math.hypot(a[0] - b[0], a[1] - b[1])
    const r = canvasEl.value.getBoundingClientRect()
    if (pinchDist) zoomAt(d / pinchDist, (a[0] + b[0]) / 2 - r.left, (a[1] + b[1]) / 2 - r.top)
    pinchDist = d
  }
}

function onUp(e) {
  if (drag) {
    emit('pick', { x: drag.x, y: drag.y, yaw: drag.yaw })
    drag = null
    needDraw = true
    return
  }
  pointers.delete(e.pointerId)
  pinchDist = 0
}

let ro = null
onMounted(() => {
  ro = new ResizeObserver(resize)
  ro.observe(wrapEl.value)
  loadGrid()
  raf = requestAnimationFrame(loop)
})
onUnmounted(() => { cancelAnimationFrame(raf); ro?.disconnect() })

defineExpose({ fitMap })
</script>

<template>
  <div class="wrap">
    <div class="tools">
      <label><input v-model="follow" type="checkbox" /> 跟随车辆</label>
      <button @click="fitMap">全图</button>
      <span v-if="mode === 'goal'" class="hint goal">在地图上按下选择目标点，拖动设置车头朝向</span>
      <span v-else-if="mode === 'reloc'" class="hint reloc">在地图上按下选择车的大概位置，拖动设置车头朝向</span>
    </div>
    <div ref="wrapEl" class="canvas-wrap" :class="mode">
      <canvas
        ref="canvasEl"
        @wheel="onWheel"
        @pointerdown="onDown"
        @pointermove="onMove"
        @pointerup="onUp"
        @pointercancel="onUp"
      />
      <div class="legend">
        <span><i class="sw" style="background:#2563eb" />车</span>
        <span><i class="sw" style="background:#16a34a" />规划路径</span>
        <span><i class="sw" style="background:#e11d48" />实时障碍</span>
        <span><i class="sw dash" />建图轨迹</span>
      </div>
    </div>
  </div>
</template>

<style scoped>
.wrap { display: flex; flex-direction: column; height: 100%; gap: 8px; }
.tools { display: flex; align-items: center; gap: 12px; flex-wrap: wrap; font-size: 13px; }
.tools label { display: flex; align-items: center; gap: 4px; cursor: pointer; }
.tools button {
  border: 1px solid var(--line); background: transparent; color: var(--text);
  border-radius: 6px; padding: 3px 10px; cursor: pointer; font-size: 13px;
}
.hint { padding: 2px 10px; border-radius: 10px; color: #fff; font-size: 12px; }
.hint.goal { background: #d97706; }
.hint.reloc { background: #9333ea; }
.canvas-wrap { position: relative; flex: 1; min-height: 320px; border-radius: 8px; overflow: hidden; }
.canvas-wrap.goal, .canvas-wrap.reloc { outline: 3px solid #d97706; }
.canvas-wrap.reloc { outline-color: #9333ea; }
canvas { position: absolute; inset: 0; width: 100%; height: 100%; touch-action: none; cursor: grab; }
.goal canvas, .reloc canvas { cursor: crosshair; }
.legend {
  position: absolute; right: 10px; bottom: 10px; display: flex; gap: 10px; flex-wrap: wrap;
  font-size: 11px; color: #111; background: rgba(255,255,255,0.8); padding: 3px 8px; border-radius: 6px;
}
.sw { display: inline-block; width: 10px; height: 10px; border-radius: 2px; margin-right: 4px; vertical-align: -1px; }
.sw.dash { background: repeating-linear-gradient(90deg, #7878c8 0 4px, transparent 4px 7px); }
</style>
