import { describe, expect, test } from 'bun:test'
import { isCommand } from './commands'
import { applyMockCommand, initialState, tickMock } from './state'

describe('command validation', () => {
  test('accepts valid commands', () => {
    expect(isCommand({ type: 'manual', robot: 'RMC1', x: 0.4, y: 0.2, yaw: 0 })).toBe(true)
    expect(isCommand({ type: 'lift', robot: 'RMC2', height: 0.1 })).toBe(true)
    expect(isCommand({ type: 'gripper', robot: 'RMC1', state: 'closed' })).toBe(true)
  })

  test('rejects wrong robot tools and non-finite coordinates', () => {
    expect(isCommand({ type: 'lift', robot: 'RMC1', height: 0.1 })).toBe(false)
    expect(isCommand({ type: 'gripper', robot: 'RMC2', state: 'open' })).toBe(false)
    expect(isCommand({ type: 'set_goal', robot: 'RMC1', pose: { x: Number.NaN, y: 0, yaw: 0 } })).toBe(false)
  })
})

describe('mock state', () => {
  test('emergency stop blocks motion and tool state is retained', () => {
    const state = initialState('mock')
    applyMockCommand(state, { type: 'manual', robot: 'RMC2', x: 0.5, y: 0, yaw: 0 })
    tickMock(state, 1)
    expect(state.robots.RMC2.pose.x).toBeCloseTo(0.5)

    applyMockCommand(state, { type: 'lift', robot: 'RMC2', height: 0.1 })
    applyMockCommand(state, { type: 'emergency', robot: 'RMC2', active: true })
    expect(state.robots.RMC2.velocity.x).toBe(0)
    expect(state.robots.RMC2.liftStatus).toBe('up')
  })
})
