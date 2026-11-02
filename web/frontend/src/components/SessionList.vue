<script setup>
import { ref } from 'vue'
import { makeMature } from '../api.js'

defineProps({
  sessions: { type: Array, required: true },
  dataRoot: { type: String, default: '' },
})

const STATUS = { done: '完成', failed: '失败', recording: '进行中' }
const hms = s => [s / 3600, (s / 60) % 60, s % 60].map(v => String(Math.floor(v)).padStart(2, '0')).join(':')
const toTime = n => n.replace(/^(\d{4})(\d\d)(\d\d)_(\d\d)(\d\d)(\d\d)$/, '$2-$3 $4:$5')
const msg = ref('')

// s.mature = { map: meta.json 或 null, progress: 最近一次生成的进度 或 null }
const mapOf = s => s.mature?.map
const progressOf = s => s.mature?.progress || {}

async function promote(s) {
  const existing = mapOf(s)
  if (existing && !confirm(
    `重新生成成熟地图「${existing.name}」？\n\n` +
    `会用当前配置重新处理这次录制，约需录制时长的 1~1.5 倍。\n` +
    `生成完成前，原有地图照常可用；生成失败也不会影响原有地图。`)) return
  const name = prompt(existing ? '地图名称：' : `把 ${toTime(s.name)} 的录制设为成熟地图。\n地图名称：`,
    existing?.name || progressOf(s).name || '')
  if (!name || !name.trim()) return
  const r = await makeMature(s.name, name.trim())
  msg.value = r.msg
}

const mapTitle = m => `${m.name}\n生成于 ${m.created}\n轨迹 ${m.path_length_m} m`
</script>

<template>
  <div class="card">
    <div class="title">录制记录</div>
    <div class="muted root">{{ dataRoot }}</div>
    <table>
      <thead>
        <tr><th>开始</th><th>时长</th><th>原始包</th><th /><th>成熟地图</th></tr>
      </thead>
      <tbody>
        <tr v-for="s in sessions" :key="s.name">
          <td class="num" :title="s.name">{{ toTime(s.name) }}</td>
          <td class="num">{{ s.duration_s ? hms(Number(s.duration_s)) : '-' }}</td>
          <td class="num">{{ s.bag_size || '-' }}</td>
          <td><span class="tag" :class="s.status" :title="s.error || ''">{{ STATUS[s.status] || s.status }}</span></td>
          <td><div class="mature-cell">
            <span v-if="mapOf(s)" class="tag mature" :title="mapTitle(mapOf(s))">{{ mapOf(s).name }}</span>
            <span v-if="progressOf(s).status === 'running'" class="tag running" :title="progressOf(s).step">
              {{ mapOf(s) ? '重新生成中…' : '生成中…' }}</span>
            <template v-else-if="s.status === 'done' && s.has_bag">
              <span v-if="progressOf(s).status === 'failed'" class="tag failed" :title="progressOf(s).step">
                {{ mapOf(s) ? '重新生成失败' : '生成失败' }}</span>
              <button class="link" @click="promote(s)">
                {{ mapOf(s) ? '重新生成' : progressOf(s).status === 'failed' ? '重试' : '设为成熟地图' }}
              </button>
            </template>
          </div></td>
        </tr>
        <tr v-if="!sessions.length"><td colspan="5" class="muted">暂无记录</td></tr>
      </tbody>
    </table>
    <div v-if="msg" class="muted msg">{{ msg }}</div>
  </div>
</template>

<style scoped>
.title { font-weight: 600; }
.root { font-size: 12px; margin-bottom: 6px; word-break: break-all; }
table { width: 100%; border-collapse: collapse; font-size: 12px; }
th, td { text-align: left; padding: 6px 4px; border-bottom: 1px solid var(--line); }
th { color: var(--muted); font-weight: 500; }
.tag { padding: 1px 6px; border-radius: 4px; background: var(--line); white-space: nowrap; }
.tag.failed { background: var(--stop); color: #fff; }
.tag.mature { background: var(--go); color: #fff; }
.mature-cell { display: flex; flex-wrap: wrap; gap: 4px; align-items: center; }
.tag.running { background: var(--warn); color: #fff; }
.link {
  border: 1px solid var(--line); background: transparent; color: var(--text); border-radius: 4px;
  padding: 1px 6px; font-size: 12px; cursor: pointer; white-space: nowrap;
}
.msg { font-size: 12px; margin-top: 8px; }
</style>
