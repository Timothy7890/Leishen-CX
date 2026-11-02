async function post(url, body = {}) {
  const r = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  })
  return r.json().catch(() => ({ ok: false, msg: `HTTP ${r.status}` }))
}

export const listMaps = () => fetch('api/maps').then(r => r.json())
export const mapTraj = session => fetch(`api/maps/${session}/traj`).then(r => r.json())
export const gridUrl = session => `api/maps/${session}/grid.png`
export const navStatus = () => fetch('api/nav/status').then(r => r.json())
export const navStart = (session, dryRun) => post('api/nav/start', { session, dry_run: dryRun })
export const navStop = () => post('api/nav/stop')
export const navCmd = (cmd, body) => post(`api/nav/${cmd}`, body)
export const streamUrl = 'api/nav/stream'
