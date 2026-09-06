import { dirname, join } from 'node:path'
import type { Server, ServerWebSocket } from 'bun'
import type { Command, FmsState } from '../src/types'
import { isCommand } from './commands'
import { applyMockCommand, initialState, tickMock } from './state'

type WsData = { connectedAt: number }

const moduleDir = dirname(dirname(import.meta.path))
const port = Number(process.env.PORT ?? (process.env.NODE_ENV === 'development' ? 3001 : 3000))
const mock = process.env.MOCK_ROS === '1'
let state = initialState(mock ? 'mock' : 'ros')
let bridge: Bun.Subprocess<'pipe', 'pipe', 'inherit'> | null = null

function json(data: unknown, status = 200): Response {
  return Response.json(data, { status, headers: { 'cache-control': 'no-store' } })
}

function sendToBridge(command: Command): boolean {
  if (mock) {
    applyMockCommand(state, command)
    return true
  }
  if (!bridge?.stdin) return false
  bridge.stdin.write(`${JSON.stringify(command)}\n`)
  bridge.stdin.flush()
  return true
}

function broadcast(server: Server<WsData>): void {
  server.publish('telemetry', JSON.stringify(state))
}

async function readBridge(server: Server<WsData>): Promise<void> {
  if (!bridge?.stdout) return
  const decoder = new TextDecoder()
  let pending = ''
  for await (const chunk of bridge.stdout) {
    pending += decoder.decode(chunk, { stream: true })
    const lines = pending.split('\n')
    pending = lines.pop() ?? ''
    for (const line of lines) {
      if (!line.trim()) continue
      try {
        const next = JSON.parse(line) as FmsState
        if (next.type === 'state') {
          state = next
          state.mode = 'ros'
          state.bridgeOnline = true
          state.bridgeError = null
          broadcast(server)
        }
      } catch {
        console.error(`[ROS bridge] invalid JSON: ${line.slice(0, 180)}`)
      }
    }
  }
  state.bridgeOnline = false
  state.bridgeError = `ROS bridge stopped (code ${await bridge.exited})`
  broadcast(server)
}

function startBridge(server: Server<WsData>): void {
  if (mock) {
    setInterval(() => {
      tickMock(state)
      broadcast(server)
    }, 100)
    return
  }

  bridge = Bun.spawn(['python3', join(moduleDir, 'server', 'ros_bridge.py')], {
    cwd: moduleDir,
    stdin: 'pipe',
    stdout: 'pipe',
    stderr: 'inherit',
    env: { ...process.env, PYTHONUNBUFFERED: '1' },
  })
  void readBridge(server)
}

const server = Bun.serve<WsData>({
  port,
  hostname: '0.0.0.0',
  async fetch(request, server) {
    const url = new URL(request.url)
    if (url.pathname === '/ws') {
      return server.upgrade(request, { data: { connectedAt: Date.now() } })
        ? undefined
        : new Response('WebSocket upgrade failed', { status: 400 })
    }
    if (url.pathname === '/api/health') {
      return json({ ok: true, mode: state.mode, bridgeOnline: state.bridgeOnline, timestamp: Date.now() })
    }
    if (url.pathname === '/api/state') return json(state)
    if (url.pathname === '/api/command' && request.method === 'POST') {
      let body: unknown
      try {
        body = await request.json()
      } catch {
        return json({ ok: false, error: 'invalid JSON' }, 400)
      }
      if (!isCommand(body)) return json({ ok: false, error: 'invalid command' }, 400)
      if (!sendToBridge(body)) return json({ ok: false, error: 'ROS bridge is offline' }, 503)
      broadcast(server)
      return json({ ok: true })
    }

    const relative = url.pathname === '/' ? 'index.html' : url.pathname.slice(1)
    if (relative.includes('..')) return new Response('Bad path', { status: 400 })
    const file = Bun.file(join(moduleDir, 'dist', relative))
    if (await file.exists()) return new Response(file)
    const fallback = Bun.file(join(moduleDir, 'dist', 'index.html'))
    return (await fallback.exists()) ? new Response(fallback) : new Response('Run bun run build first', { status: 404 })
  },
  websocket: {
    open(ws) {
      ws.subscribe('telemetry')
      ws.send(JSON.stringify(state))
    },
    message(ws: ServerWebSocket<WsData>, message) {
      try {
        const command: unknown = JSON.parse(String(message))
        if (isCommand(command)) sendToBridge(command)
      } catch {
        ws.send(JSON.stringify({ type: 'error', message: 'invalid command' }))
      }
    },
  },
})

startBridge(server)
console.log(`MVCH FMS: http://0.0.0.0:${port} (${mock ? 'mock' : 'ROS'})`)
