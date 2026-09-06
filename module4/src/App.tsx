import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import type { Command, FmsState, Point, Pose, RobotId, RobotState } from './types'

const WORLD = { minX: -5.5, maxX: 0.5, minY: -0.5, maxY: 5.5 }
const SVG_SIZE = 600

const emptyRobot: RobotState = {
  online: false,
  lastMessageMs: null,
  pose: { x: 0, y: 0, yaw: 0 },
  velocity: { x: 0, y: 0, yaw: 0 },
  battery: null,
  lidar: [],
  trail: [],
  plan: [],
  goal: null,
  emergency: false,
  arucoId: null,
  liftStatus: null,
  gripper: 'unknown',
}

const initialState: FmsState = {
  type: 'state',
  timestamp: 0,
  mode: 'ros',
  bridgeOnline: false,
  bridgeError: null,
  map: null,
  robots: { RMC1: emptyRobot, RMC2: emptyRobot },
}

function worldToSvg(point: Point): Point {
  return {
    x: ((point.y - WORLD.minY) / (WORLD.maxY - WORLD.minY)) * SVG_SIZE,
    y: ((point.x - WORLD.minX) / (WORLD.maxX - WORLD.minX)) * SVG_SIZE,
  }
}

function points(pointsValue: Point[]): string {
  return pointsValue.map(worldToSvg).map((point) => `${point.x},${point.y}`).join(' ')
}

function format(value: number | null, digits = 2): string {
  return value == null || !Number.isFinite(value) ? '—' : value.toFixed(digits)
}

function Card({ title, children, className = '' }: { title: string; children: React.ReactNode; className?: string }) {
  return (
    <section className={`rounded-xl border border-slate-700 bg-slate-900/80 p-4 ${className}`}>
      <h2 className="mb-3 text-sm font-semibold uppercase tracking-wide text-slate-400">{title}</h2>
      {children}
    </section>
  )
}

function NumberField({ label, value, onChange }: { label: string; value: number; onChange: (value: number) => void }) {
  return (
    <label className="grid gap-1 text-xs text-slate-400">
      {label}
      <input
        className="w-full rounded-lg border border-slate-700 bg-slate-950 px-2 py-2 text-sm text-slate-100 outline-none focus:border-cyan-500"
        type="number"
        step="0.1"
        value={value}
        onChange={(event) => onChange(Number(event.target.value))}
      />
    </label>
  )
}

function FieldMap({ state, selected }: { state: FmsState; selected: RobotId }) {
  const occupied = useMemo(() => {
    if (!state.map) return []
    const result: { x: number; y: number; size: number; value: number }[] = []
    for (let row = 0; row < state.map.height; row += 1) {
      for (let col = 0; col < state.map.width; col += 1) {
        const value = state.map.data[row * state.map.width + col]
        if (value > 25) {
          result.push({
            x: state.map.origin.x + col * state.map.resolution,
            y: state.map.origin.y + row * state.map.resolution,
            size: state.map.resolution,
            value,
          })
        }
      }
    }
    return result.slice(0, 5000)
  }, [state.map])

  const markers = Array.from({ length: 36 }, (_, id) => ({
    id,
    x: -Math.floor(id / 6),
    y: id % 6,
  }))

  return (
    <div className="relative aspect-square min-h-[420px] overflow-hidden rounded-xl border border-slate-700 bg-slate-950">
      <svg viewBox={`0 0 ${SVG_SIZE} ${SVG_SIZE}`} className="h-full w-full" aria-label="Карта поля и телеметрия роверов">
        <rect width={SVG_SIZE} height={SVG_SIZE} fill="#020617" />
        {occupied.map((cell, index) => {
          const topLeft = worldToSvg({ x: cell.x, y: cell.y })
          const bottomRight = worldToSvg({ x: cell.x + cell.size, y: cell.y + cell.size })
          return (
            <rect
              key={`cell-${index}`}
              x={Math.min(topLeft.x, bottomRight.x)}
              y={Math.min(topLeft.y, bottomRight.y)}
              width={Math.max(1, Math.abs(bottomRight.x - topLeft.x))}
              height={Math.max(1, Math.abs(bottomRight.y - topLeft.y))}
              fill={cell.value > 65 ? '#334155' : '#1e293b'}
              opacity="0.7"
            />
          )
        })}

        {Array.from({ length: 6 }, (_, row) => Array.from({ length: 6 }, (_, col) => {
          const here = worldToSvg({ x: -row, y: col })
          const right = col < 5 ? worldToSvg({ x: -row, y: col + 1 }) : null
          const down = row < 5 ? worldToSvg({ x: -(row + 1), y: col }) : null
          return (
            <g key={`edges-${row}-${col}`} stroke="#1e3a5f" strokeWidth="2">
              {right && <line x1={here.x} y1={here.y} x2={right.x} y2={right.y} />}
              {down && <line x1={here.x} y1={here.y} x2={down.x} y2={down.y} />}
            </g>
          )
        }))}

        {(['RMC1', 'RMC2'] as RobotId[]).map((id) => {
          const rover = state.robots[id]
          const color = id === 'RMC1' ? '#22d3ee' : '#f59e0b'
          const center = worldToSvg(rover.pose)
          const direction = rover.pose.yaw - Math.PI / 2
          return (
            <g key={id} opacity={id === selected ? 1 : 0.55}>
              {rover.trail.length > 1 && <polyline points={points(rover.trail)} fill="none" stroke={color} strokeWidth="3" opacity="0.45" />}
              {rover.plan.length > 1 && <polyline points={points(rover.plan)} fill="none" stroke="#a78bfa" strokeWidth="4" strokeDasharray="10 7" />}
              {rover.lidar.map((point, index) => {
                const p = worldToSvg(point)
                return <circle key={`${id}-lidar-${index}`} cx={p.x} cy={p.y} r="2.2" fill={color} opacity="0.55" />
              })}
              {rover.goal && (() => {
                const goal = worldToSvg(rover.goal)
                return <g><circle cx={goal.x} cy={goal.y} r="12" fill="none" stroke="#a78bfa" strokeWidth="3" /><line x1={goal.x - 16} y1={goal.y} x2={goal.x + 16} y2={goal.y} stroke="#a78bfa" /><line x1={goal.x} y1={goal.y - 16} x2={goal.x} y2={goal.y + 16} stroke="#a78bfa" /></g>
              })()}
              <g transform={`translate(${center.x} ${center.y}) rotate(${direction * 180 / Math.PI})`}>
                <circle r={id === selected ? 18 : 14} fill={color} stroke="#f8fafc" strokeWidth={id === selected ? 3 : 1} />
                <path d="M 0 -25 L -7 -10 L 7 -10 Z" fill="#f8fafc" />
              </g>
              <text x={center.x + 21} y={center.y - 18} fill={color} fontSize="14" fontWeight="700">{id}</text>
            </g>
          )
        })}

        {markers.map((marker) => {
          const point = worldToSvg(marker)
          const current = state.robots.RMC2.arucoId === String(marker.id)
          return (
            <g key={marker.id}>
              <rect x={point.x - 11} y={point.y - 11} width="22" height="22" rx="4" fill={current ? '#f59e0b' : '#0f172a'} stroke={current ? '#fbbf24' : '#475569'} strokeWidth="2" />
              <text x={point.x} y={point.y + 4} textAnchor="middle" fill="#f8fafc" fontSize="10">{marker.id}</text>
            </g>
          )
        })}
      </svg>
      <div className="absolute left-3 top-3 rounded-lg bg-slate-950/80 px-3 py-2 text-xs text-slate-300 backdrop-blur">
        ArUco: шаг 1 м · X вниз · Y вправо
      </div>
      <div className="absolute bottom-3 right-3 flex gap-3 rounded-lg bg-slate-950/80 px-3 py-2 text-xs backdrop-blur">
        <span className="text-cyan-300">● RMC1</span><span className="text-amber-300">● RMC2</span><span className="text-violet-300">-- маршрут</span>
      </div>
    </div>
  )
}

function ManualControls({ robot, emergency, send }: { robot: RobotId; emergency: boolean; send: (command: Command) => void }) {
  const timer = useRef<ReturnType<typeof setInterval> | null>(null)

  const stop = useCallback(() => {
    if (timer.current) clearInterval(timer.current)
    timer.current = null
    send({ type: 'stop', robot })
  }, [robot, send])

  const start = (x: number, y: number, yaw: number) => {
    if (emergency) return
    if (timer.current) clearInterval(timer.current)
    const command: Command = { type: 'manual', robot, x, y, yaw }
    send(command)
    timer.current = setInterval(() => send(command), 120)
  }

  useEffect(() => stop, [stop])

  const button = 'select-none rounded-lg border border-slate-600 bg-slate-800 px-3 py-3 font-semibold text-slate-100 hover:bg-slate-700 active:bg-cyan-700 disabled:cursor-not-allowed disabled:opacity-40'
  const handlers = (x: number, y: number, yaw: number) => ({
    onPointerDown: (event: React.PointerEvent<HTMLButtonElement>) => {
      event.currentTarget.setPointerCapture(event.pointerId)
      start(x, y, yaw)
    },
    onPointerUp: stop,
    onPointerCancel: stop,
  })

  return (
    <div className="grid grid-cols-3 gap-2">
      <button className={button} disabled={robot === 'RMC2' || emergency} {...handlers(0, 0.35, 0)}>↖ бок</button>
      <button className={button} disabled={emergency} {...handlers(0.45, 0, 0)}>↑ W</button>
      <button className={button} disabled={robot === 'RMC2' || emergency} {...handlers(0, -0.35, 0)}>бок ↗</button>
      <button className={button} disabled={emergency} {...handlers(0, 0, 0.75)}>↶ A</button>
      <button className={`${button} border-red-700 bg-red-950 text-red-200`} onClick={stop}>STOP</button>
      <button className={button} disabled={emergency} {...handlers(0, 0, -0.75)}>D ↷</button>
      <button className={button} disabled={robot === 'RMC2' || emergency} {...handlers(0, -0.35, 0)}>↙ бок</button>
      <button className={button} disabled={emergency} {...handlers(-0.45, 0, 0)}>↓ S</button>
      <button className={button} disabled={robot === 'RMC2' || emergency} {...handlers(0, 0.35, 0)}>бок ↘</button>
    </div>
  )
}

export default function App() {
  const [state, setState] = useState<FmsState>(initialState)
  const [selected, setSelected] = useState<RobotId>('RMC1')
  const [socketOnline, setSocketOnline] = useState(false)
  const [message, setMessage] = useState('Подключение…')
  const [poseForm, setPoseForm] = useState<Pose>({ x: 0, y: 0, yaw: 0 })
  const [goalForm, setGoalForm] = useState<Pose>({ x: -2, y: 3, yaw: 0 })

  const send = useCallback(async (command: Command) => {
    try {
      const response = await fetch('/api/command', {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify(command),
      })
      const result = await response.json() as { ok: boolean; error?: string }
      setMessage(result.ok ? `Команда: ${command.type}` : `Ошибка: ${result.error}`)
    } catch (error) {
      setMessage(`API недоступен: ${String(error)}`)
    }
  }, [])

  useEffect(() => {
    let socket: WebSocket | null = null
    let retry: ReturnType<typeof setTimeout>
    let disposed = false
    const connect = () => {
      const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:'
      socket = new WebSocket(`${protocol}//${location.host}/ws`)
      socket.onopen = () => { setSocketOnline(true); setMessage('Телеметрия подключена') }
      socket.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data) as FmsState
          if (data.type === 'state') setState(data)
        } catch { /* ignore malformed packet */ }
      }
      socket.onclose = () => {
        setSocketOnline(false)
        if (!disposed) retry = setTimeout(connect, 1200)
      }
    }
    connect()
    return () => { disposed = true; clearTimeout(retry); socket?.close() }
  }, [])

  useEffect(() => {
    const pressed = new Set<string>()
    const movement = () => {
      let x = 0, y = 0, yaw = 0
      if (pressed.has('w')) x += 0.45
      if (pressed.has('s')) x -= 0.45
      if (pressed.has('a')) yaw += 0.75
      if (pressed.has('d')) yaw -= 0.75
      if (selected === 'RMC1' && pressed.has('q')) y += 0.35
      if (selected === 'RMC1' && pressed.has('e')) y -= 0.35
      if (x || y || yaw) send({ type: 'manual', robot: selected, x, y, yaw })
      else send({ type: 'stop', robot: selected })
    }
    const down = (event: KeyboardEvent) => {
      if ((event.target as HTMLElement).tagName === 'INPUT') return
      const key = event.key.toLowerCase()
      if (!['w', 'a', 's', 'd', 'q', 'e'].includes(key)) return
      event.preventDefault()
      pressed.add(key)
      movement()
    }
    const up = (event: KeyboardEvent) => {
      const key = event.key.toLowerCase()
      if (!pressed.delete(key)) return
      movement()
    }
    const repeat = setInterval(() => { if (pressed.size) movement() }, 120)
    window.addEventListener('keydown', down)
    window.addEventListener('keyup', up)
    window.addEventListener('blur', () => { pressed.clear(); movement() })
    return () => {
      clearInterval(repeat)
      window.removeEventListener('keydown', down)
      window.removeEventListener('keyup', up)
    }
  }, [selected, send])

  const rover = state.robots[selected]
  const fresh = Date.now() - state.timestamp < 1500
  const setForm = (setter: React.Dispatch<React.SetStateAction<Pose>>, key: keyof Pose, value: number) => setter((current) => ({ ...current, [key]: value }))

  return (
    <main className="min-h-screen bg-slate-950 p-3 text-slate-100 lg:p-5">
      <header className="mx-auto mb-4 flex max-w-[1600px] flex-wrap items-center justify-between gap-3">
        <div><h1 className="text-xl font-bold">MVCH FMS · модуль Г</h1><p className="text-sm text-slate-400">Управление, навигация и телеметрия RMC1 / RMC2</p></div>
        <div className="flex flex-wrap items-center gap-2 text-xs">
          <span className={`rounded-full px-3 py-1.5 ${socketOnline && fresh ? 'bg-emerald-950 text-emerald-300' : 'bg-red-950 text-red-300'}`}>WebSocket: {socketOnline && fresh ? 'online' : 'offline'}</span>
          <span className={`rounded-full px-3 py-1.5 ${state.bridgeOnline ? 'bg-emerald-950 text-emerald-300' : 'bg-amber-950 text-amber-300'}`}>Bridge: {state.bridgeOnline ? state.mode : 'offline'}</span>
          <span className="max-w-sm truncate rounded-full bg-slate-800 px-3 py-1.5 text-slate-300">{state.bridgeError ?? message}</span>
        </div>
      </header>

      <div className="mx-auto grid max-w-[1600px] gap-4 xl:grid-cols-[minmax(600px,1fr)_420px]">
        <div className="grid gap-4">
          <div className="flex gap-2">
            {(['RMC1', 'RMC2'] as RobotId[]).map((id) => (
              <button key={id} onClick={() => { setSelected(id); setPoseForm(state.robots[id].pose) }} className={`flex-1 rounded-xl border px-4 py-3 text-left ${selected === id ? 'border-cyan-500 bg-cyan-950/60' : 'border-slate-700 bg-slate-900'}`}>
                <span className="font-bold">{id}</span>
                <span className={`ml-3 text-xs ${state.robots[id].online ? 'text-emerald-400' : 'text-red-400'}`}>{state.robots[id].online ? '● ROS online' : '● нет данных'}</span>
              </button>
            ))}
          </div>
          <FieldMap state={state} selected={selected} />
          <Card title="Состояние двух роверов">
            <div className="grid grid-cols-2 gap-3 text-sm lg:grid-cols-4">
              {(['RMC1', 'RMC2'] as RobotId[]).map((id) => <div key={id} className="rounded-lg bg-slate-950 p-3"><strong>{id}</strong><div className="mt-1 text-slate-400">x {format(state.robots[id].pose.x)} · y {format(state.robots[id].pose.y)}</div><div className="text-slate-400">yaw {format(state.robots[id].pose.yaw)} rad</div></div>)}
              <div className="rounded-lg bg-slate-950 p-3"><strong>Карта</strong><div className="mt-1 text-slate-400">{state.map ? `${state.map.width} × ${state.map.height}` : 'нет /map'}</div></div>
              <div className="rounded-lg bg-slate-950 p-3"><strong>Обновление</strong><div className="mt-1 text-slate-400">{state.timestamp ? new Date(state.timestamp).toLocaleTimeString() : '—'}</div></div>
            </div>
          </Card>
        </div>

        <aside className="grid content-start gap-4">
          <Card title={`Ручное управление ${selected}`}>
            <ManualControls robot={selected} emergency={rover.emergency} send={send} />
            <p className="mt-2 text-xs text-slate-500">Клавиши: W/S — ход, A/D — поворот, Q/E — боковой ход RMC1. При отпускании отправляется STOP.</p>
          </Card>

          <button onClick={() => send({ type: 'emergency', robot: selected, active: !rover.emergency })} className={`rounded-xl border-2 px-5 py-5 text-lg font-black tracking-wide ${rover.emergency ? 'border-emerald-500 bg-emerald-950 text-emerald-300' : 'border-red-500 bg-red-950 text-red-200'}`}>
            {rover.emergency ? 'СНЯТЬ АВАРИЙНЫЙ СТОП' : 'АВАРИЙНЫЙ СТОП'}
          </button>

          <Card title="Телеметрия">
            <dl className="grid grid-cols-2 gap-x-4 gap-y-2 text-sm">
              <dt className="text-slate-400">Позиция</dt><dd>x {format(rover.pose.x)} · y {format(rover.pose.y)}</dd>
              <dt className="text-slate-400">Ориентация</dt><dd>{format(rover.pose.yaw)} rad</dd>
              <dt className="text-slate-400">Скорость</dt><dd>{format(rover.velocity.x)} м/с</dd>
              <dt className="text-slate-400">Лидар</dt><dd>{rover.lidar.length} точек</dd>
              <dt className="text-slate-400">Аккумулятор</dt><dd>{rover.battery == null ? 'нет топика' : `${format(rover.battery)} В`}</dd>
              <dt className="text-slate-400">ArUco</dt><dd>{rover.arucoId ?? '—'}</dd>
              <dt className="text-slate-400">Инструмент</dt><dd>{selected === 'RMC1' ? rover.gripper : (rover.liftStatus ?? '—')}</dd>
              <dt className="text-slate-400">Возраст данных</dt><dd>{rover.lastMessageMs == null ? '—' : `${rover.lastMessageMs} мс`}</dd>
            </dl>
          </Card>

          <Card title="Установить текущее положение">
            <div className="grid grid-cols-3 gap-2">
              <NumberField label="X, м" value={poseForm.x} onChange={(value) => setForm(setPoseForm, 'x', value)} />
              <NumberField label="Y, м" value={poseForm.y} onChange={(value) => setForm(setPoseForm, 'y', value)} />
              <NumberField label="Yaw, рад" value={poseForm.yaw} onChange={(value) => setForm(setPoseForm, 'yaw', value)} />
            </div>
            <button onClick={() => send({ type: 'set_pose', robot: selected, pose: poseForm })} className="mt-3 w-full rounded-lg bg-slate-700 px-4 py-2 font-semibold hover:bg-slate-600">Применить без движения</button>
          </Card>

          <Card title="Автономная цель Nav2">
            <div className="grid grid-cols-3 gap-2">
              <NumberField label="X, м" value={goalForm.x} onChange={(value) => setForm(setGoalForm, 'x', value)} />
              <NumberField label="Y, м" value={goalForm.y} onChange={(value) => setForm(setGoalForm, 'y', value)} />
              <NumberField label="Yaw, рад" value={goalForm.yaw} onChange={(value) => setForm(setGoalForm, 'yaw', value)} />
            </div>
            <div className="mt-3 grid grid-cols-2 gap-2">
              <button disabled={rover.emergency} onClick={() => send({ type: 'set_goal', robot: selected, pose: goalForm })} className="rounded-lg bg-violet-700 px-3 py-2 font-semibold hover:bg-violet-600 disabled:opacity-40">Ехать к точке</button>
              <button onClick={() => send({ type: 'cancel_goal', robot: selected })} className="rounded-lg bg-slate-700 px-3 py-2 font-semibold hover:bg-slate-600">Отменить</button>
            </div>
          </Card>

          <Card title="Рабочий инструмент">
            {selected === 'RMC1' ? (
              <div className="grid grid-cols-2 gap-2"><button disabled={rover.emergency} onClick={() => send({ type: 'gripper', robot: 'RMC1', state: 'open' })} className="rounded-lg bg-cyan-800 px-3 py-2 disabled:opacity-40">Открыть схват</button><button disabled={rover.emergency} onClick={() => send({ type: 'gripper', robot: 'RMC1', state: 'closed' })} className="rounded-lg bg-cyan-800 px-3 py-2 disabled:opacity-40">Закрыть схват</button></div>
            ) : (
              <div className="grid grid-cols-2 gap-2"><button disabled={rover.emergency} onClick={() => send({ type: 'lift', robot: 'RMC2', height: 0.1 })} className="rounded-lg bg-amber-800 px-3 py-2 disabled:opacity-40">Поднять лифт</button><button disabled={rover.emergency} onClick={() => send({ type: 'lift', robot: 'RMC2', height: 0 })} className="rounded-lg bg-amber-800 px-3 py-2 disabled:opacity-40">Опустить лифт</button></div>
            )}
          </Card>
        </aside>
      </div>
    </main>
  )
}

