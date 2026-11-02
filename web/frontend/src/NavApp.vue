<script setup>
import { computed, onMounted, onUnmounted, ref, watch } from 'vue'
import { gridUrl, listMaps, mapTraj, navCmd, navStart, navStatus, navStop, streamUrl } from './navApi.js'
import NavMap from './components/NavMap.vue'

const maps = ref([])
const selected = ref('')            // 录制目录名
const traj = ref([])
const nav = ref({ status: 'connecting' })
const dryRun = ref(true)
const state = ref(null)             // nav_bridge 推送
const mode = ref('view')            // view | goal | reloc
const msg = ref('')
const speed = ref(0.5)
let es = null, statusTimer = null, hbTimer = null

const map = computed(() => maps.value.find(m => m.session === selected.value))
const running = computed(() => nav.value.status === 'running')
const busy = computed(() => ['starting', 'stopping'].includes(nav.value.status))
const loc = computed(() => state.value?.loc || {})
const gate = computed(() => state.value?.gate || {})
const serial = computed(() => state.value?.serial || {})
const estop = computed(() => !!state.value?.estop)
const localized = computed(() => loc.value.state === 'localized')
// 重定位中：点击后到定位模块进入 relocalizing 之前也算，期间两个重定位按钮都不可用
const relocSent = ref(false)
const relocBusy = computed(() => relocSent.value || loc.value.state === 'relocalizing')
watch(() => loc.value.state, s => { if (s === 'relocalizing') relocSent.value = false })
// 滑块跟随实际限速（刷新页面或重启导航后不会显示成默认值）
watch(() => state.value?.speed, v => { if (v != null) speed.value = v })

const LOC_LABEL = { unlocalized: '未定位', relocalizing: '重定位中…', localized: '已定位', lost: '定位丢失' }
const GOAL_LABEL = { idle: '无目标', preview: '路线已规划，等待出发', pending: '等待执行', active: '前往目标中', succeeded: '已到达',
  aborted: '无法到达（路径不通）', canceled: '已取消', rejected: '目标被拒绝' }

async function refreshStatus() {
  try {
    nav.value = await navStatus()
    if (running.value && !selected.value) selected.value = nav.value.session?.split('/').pop()
  } catch {
    nav.value = { status: 'offline' }
  }
}

watch(selected, async s => { traj.value = s ? await mapTraj(s) : [] })

// 运行中：订阅实时状态 + 每秒心跳（页面关闭或手机息屏 5 s 后小车自动停）
watch(running, on => {
  es?.close(); es = null
  clearInterval(hbTimer)
  if (!on) { state.value = null; mode.value = 'view'; return }
  const connect = () => {
    es = new EventSource(streamUrl)
    es.addEventListener('state', e => (state.value = JSON.parse(e.data)))
    es.onerror = () => { es.close(); if (running.value) setTimeout(connect, 1500) }
  }
  connect()
  hbTimer = setInterval(() => navCmd('heartbeat'), 1000)
}, { immediate: true })

async function act(fn, pending) {
  msg.value = pending
  const r = await fn()
  msg.value = r.msg?.split('\n').pop() || (r.ok ? '' : '操作失败')
  refreshStatus()
  return r
}

const start = () => act(() => navStart(selected.value, dryRun.value), '正在启动导航（约 20 秒）…')
const stop = () => confirm('停止导航？') && act(navStop, '正在停止…')
async function reloc(body) {
  if (relocBusy.value) return
  relocSent.value = true
  const r = await act(() => navCmd('reloc', body), '')
  // 请求被拒绝，或 3 s 内定位模块没有开始，则恢复按钮
  if (!r.ok) relocSent.value = false
  else setTimeout(() => (relocSent.value = false), 3000)
}

const autoReloc = () => reloc({ mode: 'auto' })
const goalActive = computed(() => ['pending', 'active'].includes(state.value?.goal_state))
const preview = computed(() => state.value?.goal_state === 'preview')
const cancelGoal = () => act(() => navCmd('cancel'), '')
// 点选目标只规划路线；看过路线后点出发，再确认一次车才会动
const go = () => confirm('确认出发？小车将沿绿色路线前往目标') && act(() => navCmd('go'), '')
const toggleEstop = () => act(() => navCmd('estop', { on: !estop.value }), '')
const setSpeed = () => act(() => navCmd('speed', { v: speed.value }), '')

async function onPick(p) {
  const m = mode.value
  mode.value = 'view'
  if (m === 'reloc') await reloc({ mode: 'manual', ...p })
  else if (m === 'goal') await act(() => navCmd('goal', p), '')
}

const fmtPose = p => (p ? `(${p[0].toFixed(2)}, ${p[1].toFixed(2)}) ${(p[2] * 180 / Math.PI).toFixed(0)}°` : '-')

// 空格键急停
function onKey(e) {
  if (e.code === 'Space' && running.value && e.target.tagName !== 'INPUT') {
    e.preventDefault()
    if (!estop.value) toggleEstop()
  }
}

onMounted(async () => {
  maps.value = await listMaps()
  await refreshStatus()
  if (!selected.value && maps.value.length) selected.value = maps.value[0].session
  statusTimer = setInterval(refreshStatus, 2000)
  window.addEventListener('keydown', onKey)
})
onUnmounted(() => {
  clearInterval(statusTimer); clearInterval(hbTimer); es?.close()
  window.removeEventListener('keydown', onKey)
})
</script>

<template>
  <main class="layout">
    <header>
      <h1>镭神 CX 导航</h1>
      <span class="muted">成熟地图 · 自动重定位 · 点选目标 · 遇障停车</span>
    </header>

    <section class="side">
      <!-- 急停 -->
      <button v-if="running" class="estop" :class="{ on: estop }" @click="toggleEstop">
        {{ estop ? '已急停 · 点击解除' : '急 停（空格键）' }}
      </button>

      <div class="card">
        <div class="title">地图</div>
        <select v-model="selected" :disabled="running || busy">
          <option v-if="!maps.length" value="">暂无成熟地图（在 7010 页面设置）</option>
          <option v-for="m in maps" :key="m.session" :value="m.session">
            {{ m.name }}（{{ m.session }}，{{ m.path_length_m }} m）
          </option>
        </select>
        <div class="row">
          <label v-if="!running" class="chk"><input v-model="dryRun" type="checkbox" :disabled="busy" />
            空跑（不向小车发指令）</label>
          <span v-else class="tag" :class="nav.dry_run === 'true' ? 'warn' : 'live'">
            {{ nav.dry_run === 'true' ? '空跑模式' : '实车模式' }}</span>
        </div>
        <div class="btns">
          <button class="go" :disabled="running || busy || !selected || nav.recording" @click="start">启动导航</button>
          <button class="stop" :disabled="!running || busy" @click="stop">停止导航</button>
        </div>
        <div v-if="nav.recording && !running" class="warn-text">正在录制（7010），需先停止录制</div>
        <div v-if="nav.error" class="err">{{ nav.error }}</div>
      </div>

      <template v-if="running">
        <div class="card">
          <div class="title">定位 <span class="tag" :class="loc.state">{{ LOC_LABEL[loc.state] || '等待中' }}</span></div>
          <div class="muted small">{{ loc.msg }}</div>
          <div v-if="loc.fitness != null" class="muted small num">匹配度 {{ loc.fitness }} · 误差 {{ loc.rmse }} m</div>
          <div class="muted small num">位置 {{ fmtPose(state?.pose) }}</div>
          <div class="btns">
            <button :disabled="relocBusy" @click="autoReloc">{{ relocBusy ? '重定位中…' : '自动重定位' }}</button>
            <button :disabled="relocBusy" :class="{ active: mode === 'reloc' }"
                    @click="mode = mode === 'reloc' ? 'view' : 'reloc'">手动点选</button>
          </div>
        </div>

        <div class="card">
          <div class="title">行驶 <span class="tag" :class="gate.state">{{ gate.msg || '-' }}</span></div>
          <div class="muted small">目标：{{ GOAL_LABEL[state?.goal_state] || '-' }}</div>
          <div class="muted small num">当前速度 {{ gate.v ?? 0 }} m/s · {{ gate.w ?? 0 }} rad/s</div>
          <div class="btns">
            <button :disabled="!localized || estop || goalActive" :class="{ active: mode === 'goal' }"
                    @click="mode = mode === 'goal' ? 'view' : 'goal'">点选目标</button>
            <button class="go" :disabled="!preview || !localized || estop" @click="go">出发</button>
            <button :disabled="!preview && !goalActive" @click="cancelGoal">取消目标</button>
          </div>
          <div class="speed">
            <span>限速 {{ speed.toFixed(2) }} m/s</span>
            <input v-model.number="speed" type="range" min="0.1" max="0.6" step="0.05" @change="setSpeed" />
          </div>
          <div class="muted small">
            底盘：{{ serial.dry_run ? '空跑（不发送）' : !serial.connected ? '未连接'
              : serial.chassis === 'ok' ? '已连接，底盘接受指令' : '已连接 /dev/car' }}
            <span v-if="serial.error" class="err">{{ serial.error }}</span>
          </div>
        </div>
      </template>
      <div v-if="msg" class="muted small msg">{{ msg }}</div>
    </section>

    <section class="mapbox card">
      <NavMap :map="map" :grid-url="selected ? gridUrl(selected) : ''" :traj="traj"
              :state="state" :mode="mode" @pick="onPick" />
    </section>
  </main>
</template>

<style scoped>
.layout {
  display: grid; grid-template-columns: 340px 1fr; grid-template-rows: auto 1fr;
  grid-template-areas: "head head" "side map"; gap: 16px; padding: 16px; height: 100vh;
}
header { grid-area: head; display: flex; align-items: baseline; gap: 12px; flex-wrap: wrap; }
h1 { font-size: 20px; margin: 0; }
.side { grid-area: side; display: flex; flex-direction: column; gap: 12px; overflow: auto; min-height: 0; }
.mapbox { grid-area: map; min-height: 0; padding: 12px; }
.title { font-weight: 600; margin-bottom: 6px; display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
.small { font-size: 12px; margin-top: 2px; }
select { width: 100%; padding: 8px; border-radius: 8px; border: 1px solid var(--line); background: var(--card); color: var(--text); }
.row { margin: 8px 0; font-size: 13px; }
.chk { display: flex; gap: 6px; align-items: center; cursor: pointer; }
.btns { display: flex; gap: 8px; margin-top: 10px; }
.btns button {
  flex: 1; padding: 9px; border-radius: 8px; border: 1px solid var(--line); background: transparent;
  color: var(--text); font-size: 14px; cursor: pointer;
}
.btns button.active { background: #9333ea; color: #fff; border-color: #9333ea; }
.btns button:disabled { opacity: 0.35; cursor: default; }
.btns .go { background: var(--go); color: #fff; border: 0; font-weight: 600; }
.btns .stop { background: var(--stop); color: #fff; border: 0; font-weight: 600; }
.estop {
  width: 100%; padding: 18px; border: 0; border-radius: 12px; font-size: 20px; font-weight: 700;
  color: #fff; background: #dc2626; cursor: pointer; box-shadow: 0 2px 0 #991b1b;
}
.estop.on { background: #7f1d1d; animation: blink 1s infinite; }
@keyframes blink { 50% { opacity: 0.75; } }
.tag { font-size: 12px; font-weight: 500; padding: 1px 8px; border-radius: 10px; background: var(--line); }
.tag.localized, .tag.moving, .tag.live { background: var(--go); color: #fff; }
.tag.relocalizing, .tag.blocked, .tag.init, .tag.warn { background: var(--warn); color: #fff; }
.tag.lost, .tag.estop, .tag.no_loc, .tag.no_heartbeat, .tag.no_scan { background: var(--stop); color: #fff; }
.speed { display: flex; align-items: center; gap: 10px; margin: 10px 0 4px; font-size: 13px; }
.speed input { flex: 1; }
.err { color: var(--stop); font-size: 12px; margin-top: 6px; white-space: pre-wrap; }
.warn-text { color: var(--warn); font-size: 12px; margin-top: 6px; }
.msg { white-space: pre-wrap; }

@media (max-width: 860px) {
  .layout { grid-template-columns: 1fr; grid-template-rows: auto auto 75vh; grid-template-areas: "head" "side" "map"; height: auto; }
  .side { overflow: visible; }
}
</style>
