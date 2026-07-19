import { useMemo, useRef, useState } from 'react'
import { Activity, Check, Copy, Crosshair, Focus, Footprints, Pause, Play, RotateCcw } from 'lucide-react'
import { LEGS, type LegId } from './kinematics'
import {
  GAIT_ORDER,
  PATH_POINT_LABELS,
  RobotViewport,
  type GaitSettings,
  type GaitTelemetry,
  type ViewerHandle,
  type ViewerTelemetry,
} from './RobotViewport'

const EMPTY_TELEMETRY: ViewerTelemetry = {
  converged: false,
  error: Infinity,
  iterations: 0,
  joints: [],
  actual: [0, 0, 0],
  target: [0, 0, 0],
  loaded: false,
  solving: false,
}

function formatMetres(value: number) {
  return Number.isFinite(value) ? value.toFixed(3) : '—'
}

function App() {
  const [leg, setLeg] = useState<LegId>('front_left')
  const [telemetry, setTelemetry] = useState(EMPTY_TELEMETRY)
  const [copied, setCopied] = useState(false)
  const [gaitMode, setGaitMode] = useState<'pose' | 'walk'>('pose')
  const [gaitPlaying, setGaitPlaying] = useState(false)
  const [physicsEnabled, setPhysicsEnabled] = useState(true)
  const [gaitTelemetry, setGaitTelemetry] = useState<GaitTelemetry>({
    phase: 0,
    swingLeg: 'front_left',
    contacts: 4,
    bodyOffset: 0,
    roll: 0,
    pitch: 0,
    distance: 0,
    speed: 0,
  })
  const viewerRef = useRef<ViewerHandle>(null)

  const gait = useMemo<GaitSettings>(() => ({
    mode: gaitMode,
    playing: gaitPlaying,
    speed: 0.65,
    physics: physicsEnabled,
  }), [gaitMode, gaitPlaying, physicsEnabled])

  const command = useMemo(() => ({
    leg,
    target_m: telemetry.target.map((value) => Number(value.toFixed(4))),
    joints_rad: Object.fromEntries(telemetry.joints.map((joint) => [joint.name, Number(joint.radians.toFixed(5))])),
    ...(gaitMode === 'walk' ? {
      gait: {
        cycle_phase: Number(gaitTelemetry.phase.toFixed(4)),
        speed_hz: 0.65,
        path: 'per-leg editable',
        physics_preview: physicsEnabled,
      },
    } : {}),
  }), [leg, telemetry, gaitMode, gaitTelemetry.phase, physicsEnabled])

  const updateCoordinate = (axis: number, value: number) => {
    const next = [...telemetry.target] as [number, number, number]
    next[axis] = value
    viewerRef.current?.setTarget(next)
  }

  const copyCommand = async () => {
    await navigator.clipboard.writeText(JSON.stringify(command, null, 2))
    setCopied(true)
    window.setTimeout(() => setCopied(false), 1400)
  }

  const status = !telemetry.loaded
    ? 'Loading model'
    : gaitMode === 'walk' && gaitPlaying
      ? `${LEGS[gaitTelemetry.swingLeg].code} swing phase`
      : gaitMode === 'walk'
        ? 'Walk sequence paused'
    : telemetry.converged
      ? 'Target reached'
      : telemetry.error < 0.03
        ? 'Near joint limit'
        : 'Outside workspace'

  return (
    <main className="app-shell">
      <header className="topbar">
        <div className="brand-lockup">
          <div className="brand-mark" aria-hidden="true"><span /><span /><span /><span /></div>
          <div>
            <p className="eyebrow">Robodog control systems</p>
            <h1>Inverse kinematics lab</h1>
          </div>
        </div>
        <div className={`system-state ${telemetry.converged ? 'is-good' : ''}`}>
          <span className="state-light" />
          <span>{status}</span>
          <span className="state-detail">{telemetry.loaded ? `${(telemetry.error * 1000).toFixed(1)} mm error` : 'URDF + meshes'}</span>
        </div>
      </header>

      <section className="workspace">
        <div className="viewport-panel">
          <div className="viewport-toolbar">
            <div className="leg-tabs" role="group" aria-label="Select a leg">
              {(Object.keys(LEGS) as LegId[]).map((id) => (
                <button key={id} className={id === leg ? 'is-active' : ''} onClick={() => setLeg(id)}>
                  <span>{LEGS[id].code}</span>
                  {LEGS[id].label}
                </button>
              ))}
            </div>
            <button className="icon-action" onClick={() => viewerRef.current?.resetCamera()} aria-label="Reset camera">
              <Focus size={18} />
            </button>
          </div>

          <div className="viewport">
            <RobotViewport ref={viewerRef} leg={leg} gait={gait} onTelemetry={setTelemetry} onGaitUpdate={setGaitTelemetry} />
            <div className="axis-key" aria-label="Axis colors: X red, Y green, Z blue">
              <span className="x">X</span><span className="y">Y</span><span className="z">Z</span>
            </div>
            <div className="drag-hint">
              {gaitMode === 'walk' ? <Footprints size={16} /> : <Crosshair size={16} />}
              {gaitMode === 'walk' ? 'Select and drag the path nodes' : 'Drag the amber contact target'}
            </div>
          </div>

          <div className="viewport-footer">
            <span>Orbit: left drag</span><span>Pan: right drag</span><span>Zoom: wheel</span>
            <strong>Body frame · metres · Z up</strong>
          </div>
        </div>

        <aside className="inspector">
          <div className="mode-tabs" role="group" aria-label="Motion mode">
            <button
              className={gaitMode === 'pose' ? 'is-active' : ''}
              aria-pressed={gaitMode === 'pose'}
              onClick={() => {
                setGaitPlaying(false)
                setGaitMode('pose')
              }}
            >Pose target</button>
            <button
              className={gaitMode === 'walk' ? 'is-active' : ''}
              aria-pressed={gaitMode === 'walk'}
              onClick={() => {
                setGaitPlaying(false)
                setGaitMode('walk')
              }}
            >Walk sequence</button>
          </div>

          {gaitMode === 'pose' ? <section className="inspector-section target-section">
            <div className="section-heading">
              <div>
                <p className="eyebrow">Contact target</p>
                <h2>{LEGS[leg].label}</h2>
              </div>
              <button className="text-action" onClick={() => viewerRef.current?.homeTarget()}>
                <Crosshair size={15} /> Use foot position
              </button>
            </div>

            <div className="coordinates">
              {(['X', 'Y', 'Z'] as const).map((axis, index) => (
                <label key={axis} className={`coordinate axis-${axis.toLowerCase()}`}>
                  <span>{axis}</span>
                  <input
                    type="number"
                    step="0.01"
                    value={formatMetres(telemetry.target[index])}
                    onChange={(event) => updateCoordinate(index, Number(event.target.value))}
                    disabled={!telemetry.loaded}
                  />
                  <small>m</small>
                </label>
              ))}
            </div>
            <div className="target-readout">
              <span>Actual foot</span>
              <code>{telemetry.actual.map(formatMetres).join('  ')}</code>
            </div>
          </section> : <section className="inspector-section gait-section">
            <div className="section-heading compact">
              <div>
                <p className="eyebrow">Four-beat crawl</p>
                <h2>Walking cycle</h2>
              </div>
              <span className={`solve-chip ${gaitPlaying ? 'is-good' : 'is-warn'}`}>
                {gaitPlaying ? 'Running' : 'Paused'}
              </span>
            </div>

            <div className="gait-phase" aria-label={`Gait phase ${Math.round(gaitTelemetry.phase * 100)} percent`}>
              <div className="phase-progress"><span style={{ width: `${gaitTelemetry.phase * 100}%` }} /></div>
              <div className="phase-legs">
                {GAIT_ORDER.map((id, index) => (
                  <div key={id} className={gaitTelemetry.swingLeg === id && gaitPlaying ? 'is-swing' : ''}>
                    <span>{LEGS[id].code}</span>
                    <small>{index + 1}</small>
                  </div>
                ))}
              </div>
            </div>

            <button className="primary-action gait-play" onClick={() => setGaitPlaying((playing) => !playing)} disabled={!telemetry.loaded}>
              {gaitPlaying ? <Pause size={17} /> : <Play size={17} />}
              {gaitPlaying ? 'Pause walking cycle' : 'Play walking cycle'}
            </button>

            <div className="physics-preview">
              <button
                className={`secondary-action physics-toggle ${physicsEnabled ? 'is-active' : ''}`}
                aria-pressed={physicsEnabled}
                onClick={() => setPhysicsEnabled((enabled) => !enabled)}
              >
                <Activity size={16} /> Physics preview {physicsEnabled ? 'on' : 'off'}
              </button>
              {physicsEnabled && (
                <div className="physics-readout" aria-label="Physics preview telemetry">
                  <div><span>Contacts</span><strong>{gaitTelemetry.contacts} / 4</strong></div>
                  <div><span>Forward</span><strong>{gaitTelemetry.distance.toFixed(2)} m</strong></div>
                  <div><span>Speed</span><strong>{gaitTelemetry.speed.toFixed(2)} m/s</strong></div>
                  <div><span>Roll / pitch</span><strong>{gaitTelemetry.roll.toFixed(1)}&deg; / {gaitTelemetry.pitch.toFixed(1)}&deg;</strong></div>
                </div>
              )}
            </div>

            <div className="path-editor-guide">
              <div className="path-guide-heading">
                <div>
                  <span>Editing path</span>
                  <strong>{LEGS[leg].label}</strong>
                </div>
                <button className="text-action" onClick={() => viewerRef.current?.resetPath()}>
                  <RotateCcw size={14} /> Reset path
                </button>
              </div>
              <div className="path-node-key">
                {PATH_POINT_LABELS.map((label, index) => (
                  <div key={label}><span>{index + 1}</span><small>{label}</small></div>
                ))}
              </div>
              <p>Click a node in the 3D view, then drag its X, Y, or Z axis. X movement recruits the hip for lateral balance.</p>
            </div>

            <button className="secondary-action" onClick={() => {
              setGaitPlaying(false)
              window.requestAnimationFrame(() => {
                setGaitTelemetry({
                  phase: 0,
                  swingLeg: 'front_left',
                  contacts: 4,
                  bodyOffset: 0,
                  roll: 0,
                  pitch: 0,
                  distance: 0,
                  speed: 0,
                })
                viewerRef.current?.resetPose()
              })
            }}>
              <RotateCcw size={16} /> Stop at spring stance
            </button>
          </section>}

          <section className="inspector-section joints-section">
            <div className="section-heading compact">
              <div>
                <p className="eyebrow">Solved output</p>
                <h2>{gaitMode === 'walk' ? 'Live joint command' : 'Joint command'}</h2>
              </div>
              <span className={`solve-chip ${telemetry.converged ? 'is-good' : 'is-warn'}`}>
                {telemetry.converged ? 'Solved' : 'Limited'}
              </span>
            </div>

            <div className="joint-list">
              {telemetry.joints.length > 0 ? telemetry.joints.map((joint, index) => {
                const lowerDeg = joint.lower * 180 / Math.PI
                const upperDeg = joint.upper * 180 / Math.PI
                const progress = ((joint.degrees - lowerDeg) / (upperDeg - lowerDeg)) * 100
                return (
                  <div className="joint-row" key={joint.name}>
                    <div className="joint-index">J{index + 1}</div>
                    <div className="joint-main">
                      <div className="joint-label"><span>{joint.shortName}</span><code>{joint.radians.toFixed(4)} rad</code></div>
                      <div className="joint-track"><span style={{ width: `${Math.max(0, Math.min(100, progress))}%` }} /></div>
                      <div className="joint-limits"><span>{lowerDeg.toFixed(0)}°</span><span>{upperDeg.toFixed(0)}°</span></div>
                    </div>
                    <strong>{joint.degrees.toFixed(1)}°</strong>
                  </div>
                )
              }) : <div className="loading-lines"><span /><span /><span /></div>}
            </div>

            <button className="primary-action" onClick={copyCommand} disabled={!telemetry.loaded}>
              {copied ? <Check size={17} /> : <Copy size={17} />}
              {copied ? 'Command copied' : 'Copy joint command'}
            </button>
          </section>

          <section className="inspector-section diagnostics-section">
            <p className="eyebrow">Solver diagnostics</p>
            <div className="diagnostic-grid">
              <div><span>Position error</span><strong>{Number.isFinite(telemetry.error) ? `${(telemetry.error * 1000).toFixed(1)} mm` : '—'}</strong></div>
              <div><span>Iterations</span><strong>{telemetry.iterations}</strong></div>
              <div><span>Method</span><strong>Damped least squares</strong></div>
              <div><span>Constraints</span><strong>URDF joint limits</strong></div>
            </div>
            <button className="secondary-action" onClick={() => viewerRef.current?.resetPose()}>
              <RotateCcw size={16} /> {gaitMode === 'walk' ? 'Reset spring stance' : 'Reset minimum pose'}
            </button>
          </section>
        </aside>
      </section>
    </main>
  )
}

export default App
