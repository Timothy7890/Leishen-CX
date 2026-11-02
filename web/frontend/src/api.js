export async function getStatus() {
  const r = await fetch('api/status')
  return r.json()
}

export async function startRecording() {
  const r = await fetch('api/start', { method: 'POST' })
  return r.json()
}

export async function stopRecording() {
  const r = await fetch('api/stop', { method: 'POST' })
  return r.json()
}

export async function makeMature(session, name) {
  const r = await fetch('api/mature', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ session, name }),
  })
  return r.json()
}

export const liveUrl = 'api/live'
