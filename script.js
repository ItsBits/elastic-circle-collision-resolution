'use strict'

window.onload = main

//==============================================================================
// UNUSED
//
/* c* circle center
 * v* circle velocity
 * r* circle radius
 *
 * returns true if the circles may collide
 *         false if the circles are definitely not going to collide
 */
function broad_phase(c1, v1, r1, c2, v2, r2) {
  const c1_final = [c1[0] + v1[0], c1[1] + v1[1]]
  const c2_final = [c2[0] + v2[0], c2[1] + v2[1]]

  const c1_min = [Math.min(c1[0] - r1, c1_final[0] - r1), Math.min(c1[1] - r1, c1_final[1] - r1)]
  const c1_max = [Math.max(c1[0] + r1, c1_final[0] + r1), Math.max(c1[1] + r1, c1_final[1] + r1)]
  const c2_min = [Math.min(c2[0] - r2, c2_final[0] - r2), Math.min(c2[1] - r2, c2_final[1] - r2)]
  const c2_max = [Math.max(c2[0] + r2, c2_final[0] + r2), Math.max(c2[1] + r2, c2_final[1] + r2)]

  const a = c1_max[0] >= c2_min[0]
  const b = c1_min[0] <= c2_max[0]
  const c = c1_max[1] >= c2_min[1]
  const d = c1_min[1] <= c2_max[1]

  return a && b && c && d
}

//==============================================================================
// This is where all the magic happens
//
/* c* circle center
 * v* circle velocity
 * r* circle radius
 *
 * returns time of collision in range [0, 1]
 *
 * circle-circle collision time is equivalent to case where:
 * c1 transformed in to moving point (center: same, radius:0, velocity: v1-v2)
 * c2 transformed in to stationary circle (center: same, radius: r1+r2, velocity: 0)
 *
 * line: [x, y] = [x_p, y_p] + t*[x_v, y_v]
 * circle: (x-x_c)^2+(y-y_c)^2=r^2
 *
 * line parametrization inserted in to circle equation, solved for parameter
 * and highly simplified
 */
function circle_collision_time(c1, v1, r1, c2, v2, r2) {
  const r = r1 + r2
  const a = c1[0] - c2[0]
  const b = v1[0] - v2[0]
  const c = c1[1] - c2[1]
  const d = v1[1] - v2[1]

  const den = b*b + d*d
  const sq = b*a + d*c
  const D = sq*sq - den*(a*a + c*c - r*r)

  const t = -(sq + Math.sqrt(D)) / den

  if (!isNaN(t) && t >= 0 && t <= 1)
    return t
  else
    return 1
}

//==============================================================================
function simulate_step(circles, t) {
  let dt = t
  while (dt > 0)
    dt -= simulate_step_up_to_max_t(circles, dt)
  return t - dt
}

//==============================================================================
/*
 * box: (0,0),(1,0),(1,1),(0,1)
 */
function circle_in_box_collision_time(c, v, r) {
  let left = 1
  let right = 1
  let top = 1
  let bot = 1

  if (v[0] < 0)
    left = (r - c[0]) / v[0]
  else if (v[0] > 0)
    right = (1 - c[0] - r) / v[0]

  if (v[1] < 0)
    bot = (r - c[1]) / v[1]
  else if (v[1] > 0)
    top = (1 - c[1] - r) / v[1]

  const min_t = Math.min(left, right, top, bot)

  let collisions = []
  if (min_t != 1) {
    if (min_t == left) collisions.push('l')
    if (min_t == right) collisions.push('r')
    if (min_t == top) collisions.push('t')
    if (min_t == bot) collisions.push('b')
  }
  return [min_t, collisions]
}

//==============================================================================
// max_t must be between 0 and 1
function simulate_step_up_to_max_t(circles, max_t) {
  // find time of first collision
  let ts = []
  let min_t = max_t
  for (let i = 0, len = circles.length; i < len; ++i) {
    const a = circles[i]
    // check collision with box
    const t_box = circle_in_box_collision_time(a.c, a.v, a.r)
    min_t = Math.min(t_box[0], min_t)
    ts.push([i, -1, t_box[0], t_box[1]])
    // check collisions with other circles
    for (let ii = i + 1; ii < len; ++ii) {
      const b = circles[ii]
      const t = circle_collision_time(a.c, a.v, a.r, b.c, b.v, b.r)
      ts.push([i, ii, t])
      min_t = Math.min(t, min_t)
    }
  }

  // move circles by time of first collision
  for (let i = 0, len = circles.length; i < len; ++i) {
    let c = circles[i]
    c.c[0] += c.v[0] * min_t
    c.c[1] += c.v[1] * min_t
  }

  // resolve collisions
  for (let i = 0, len = ts.length; i < len; ++i) {
    if (ts[i][2] != min_t || ts[i][2] == 1)
      continue

    if (ts[i][1] < 0) {
      // collision with wall
      const side = ts[i][3]
      const c1 = circles[ts[i][0]]
      // bounce off the walls
      if (side == 'l') c1.v[0] = Math.abs(c1.v[0])
      if (side == 'r') c1.v[0] = -Math.abs(c1.v[0])
      if (side == 't') c1.v[1] = -Math.abs(c1.v[1])
      if (side == 'b') c1.v[1] = Math.abs(c1.v[1])
    } else {
      // collision with cirlce
      const c1 = circles[ts[i][0]]
      const c2 = circles[ts[i][1]]
      const m1 = c1.m
      const m2 = c2.m
      const v1 = c1.v
      const v2 = c2.v
      // calculate new velocities assuming elastic collision
      const diff_m1_m2 = m1 - m2
      const diff_m2_m1 = m2 - m1
      const sum_mass = m1 + m2
      const new_v1_x = (v1[0] * diff_m1_m2 + 2 * m2 * v2[0]) / sum_mass
      const new_v1_y = (v1[1] * diff_m1_m2 + 2 * m2 * v2[1]) / sum_mass
      const new_v2_x = (v2[0] * diff_m2_m1 + 2 * m1 * v1[0]) / sum_mass
      const new_v2_y = (v2[1] * diff_m2_m1 + 2 * m1 * v1[1]) / sum_mass
      v1[0] = new_v1_x
      v1[1] = new_v1_y
      v2[0] = new_v2_x
      v2[1] = new_v2_y
    }
  }

  return min_t
}

//==============================================================================
function unit_lerp(min, max, v) {
  return (1 - v) * min + v * max;
}

//==============================================================================
function generate_world(sqrt_count) {
  let circles = []
  const min_radius = 0.1
  const max_radius = 0.4
  const max_velocity = 0.1

  for (let y = 0; y < sqrt_count; ++y)
    for (let x = 0; x < sqrt_count; ++x) {
      const r = unit_lerp(min_radius, max_radius, Math.random())
      const space = 0.5 - r
      circles.push({
        c: [x + 0.5 + unit_lerp(-space, space, Math.random()), y + 0.5 + unit_lerp(-space, space, Math.random())],
        v: [unit_lerp(-max_velocity, max_velocity, Math.random()), unit_lerp(-max_velocity, max_velocity, Math.random())],
        r: r,
        m: Math.PI * r * r / (Math.PI * max_radius * max_radius)
      })
    }

  for (let i = 0, len = circles.length; i < len; ++i) {
    circles[i].c[0] /= sqrt_count
    circles[i].c[1] /= sqrt_count
    circles[i].r /= sqrt_count
  }

  return circles
}

//==============================================================================
function step_and_draw(circles, canvas, ctx, t, tick_id) {

  const s = canvas.width // assuming NxN canvas

  ctx.clearRect(0, 0, s, s);

  for (let i = 0, len = circles.length; i < len; ++i) {
    const c = circles[i]
    const color = Math.round(Math.min(Math.max((1-c.m) * 255, 0), 255))
    ctx.beginPath()
    ctx.arc(c.c[0] * s, c.c[1] * s, c.r * s, 0, 2 * Math.PI)
    ctx.fillStyle = `rgb(${color}, ${color}, ${color})`
    ctx.fill()
    ctx.stroke()
  }

  t += simulate_step(circles, 0.1)
  ++tick_id

  document.querySelector('p').innerHTML = tick_id

  window.requestAnimationFrame(() => step_and_draw(circles, canvas, ctx, t, tick_id))
}

//==============================================================================
function main() {
  const circles = generate_world(5)
  const canvas = document.querySelector('canvas')
  const ctx = canvas.getContext('2d')
  step_and_draw(circles, canvas, ctx, 0, 0)
}
