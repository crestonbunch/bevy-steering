# Bevy Steering

This crate aims to provide steering behaviors for autonomous agents in
a Bevy game. It is not a complete navigation solution, but fits into a
hierarchy of systems that contribute to agent motion.

## Status of this crate

This crate is _usable_ but I can't guarantee it's _good_. I hope
to improve it over time, especially documentation and polish.

## When to use this crate

Use this crate for:

- Moving an agent.
- Making agents follow things.
- Keeping agents from running into things.
- Making groups of agents move naturally (boids).

This crate cannot provide:

- Navigation and pathfinding.
- Planning complex routes.
- Making decisions based on surroundings.

The best way to make use of this crate is to combine it with
other crates and systems to solve larger "planning" problems,
and then feed the output of those systems as inputs into a
Bevy Steering system. Look at `examples/maze.rs` to see how
to incorporate a pathfinding with `PathFollowing` and
`Avoid` behaviors.

## Movement

Bevy Steering provides two modes of moving:

- **`OmniDirectional`** (default). This ignores the direction
  of an entity and applies linear force to move the agent in
  any direction. This is ideal for really simple agents that
  don't have a front or back, or you wish to turn the agent
  manually.
- **`Directional`**. This assumes the entity can only move
  along the forward/backward vector and aggressively damps
  velocity in the lateral directions. The agent will turn
  to face the destination. This is ideal for character models,
  simple vehicles, etc.

## Implementation

The behaviors are implemented using the "Context Steering" approach. Agents
move along the most interesting heading, while avoiding dangerous directions.
Force is applied to maintain an agents `max_velocity`.

## Behaviors planned / implemented

- [x] Alignment
- [x] Approach
- [x] Avoid (obstacles)
- [x] Cohesion
- [x] Evasion
- [x] Flee
- [x] Path following
- [x] Pursuit
- [x] Seek
- [x] Separation
- [x] Wander

Most of the behaviors are described in this paper:
<https://www.red3d.com/cwr/papers/1999/gdc99steer.pdf>.

However, not all behaviors are implemented as described.
It is not a priority to implement all behaviors in this
paper.

## LLM (Large Language Model) disclaimer

This crate was made with the assistance of LLMs and coding agents.
They made it possible to produce this crate by cutting the amount of time
needed to implement most of the behaviors. That does not mean this crate is
"vibe-coded". Human effort went into making a clean interface. Code polish and
tests will continue to be human-driven.

You are free to submit PRs with the use of LLM tools. Please disclose when LLM 
tools are used in your PR.

