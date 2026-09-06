const bun = process.execPath
const env = { ...process.env, NODE_ENV: 'development' }
const backend = Bun.spawn([bun, 'run', 'server/index.ts'], { env, stdin: 'inherit', stdout: 'inherit', stderr: 'inherit' })
const frontend = Bun.spawn([bun, 'x', 'vite'], { env, stdin: 'inherit', stdout: 'inherit', stderr: 'inherit' })

async function stop(): Promise<void> {
  backend.kill()
  frontend.kill()
}

process.on('SIGINT', stop)
process.on('SIGTERM', stop)

const result = await Promise.race([backend.exited, frontend.exited])
await stop()
process.exit(result)

export {}
