# Launch post drafts

Post only after the GitHub repo is public. Replace `<repo link>` before posting. Post one at a time, and wait a few days between them.

---

## r/robotics

**Title:** I ran ~180k MuJoCo/PyBullet sims against analytic answers. Here are the settings that silently break results, packaged as a linter

I wanted to know how often a physics sim is wrong in ways you can't see, and why. So I ran a pendulum, a chaotic 2-link arm and a sliding box, about 180k runs in total, and compared each one with an analytic or high-precision reference.

The biggest errors weren't about the time step. They came from defaults:

- **PyBullet link damping defaults to 0.04.** An undamped pendulum loses 68% of its energy in 10 s.
- **PyBullet multiplies the friction of the two bodies.** Set μ=0.5 on both the box and the floor and you get 0.25. Leave both at the default and you also get 0.25.
- **MuJoCo takes the larger of the two frictions.** A box with μ=0.4 on a default floor (1.0) slides with μ=1.0.
- **MuJoCo tips boxes that rigid-body theory says won't tip.** Once μ·(half-height)/(half-width) goes above about 0.6, some of them roll over.

I also found that "it matches the other engine" isn't evidence. MuJoCo and PyBullet agreed with each other on the chaotic arm for 17 s, but both left the true answer after 1.3 s.

So I packaged the rules as `simdiff-check`. It reads a MuJoCo model, or records your PyBullet setup calls, and flags these settings. For each finding it shows the measurement behind it and how to fix it. It can also estimate your numerical error without ground truth by running at dt, dt/2 and dt/4.

I tested it on 8,000 new random scenes. None of the scenes that passed every rule were off by more than 1%, and every scene that was off by more than 1% got flagged. The limits: I've only validated it on sliding contact and simple articulated systems so far. Impacts, grasping and rolling aren't covered yet.

<repo link>

What I'd love to hear:
1. Which sim setting has silently burned you? I want to test more of them.
2. Does anyone have a contact scene with a known answer, like a drop test with a measured bounce, that I could validate against?

---

## r/reinforcementlearning

**Title:** Your sim-trained policy may be exploiting a default setting. I built a linter from 180k verified sims

If you train in MuJoCo or PyBullet, some defaults change the physics your policy learns:

- PyBullet adds 0.04 damping to every link.
- PyBullet multiplies friction, so two default bodies get μ=0.25.
- MuJoCo takes the larger friction of the two bodies, so a floor you never touched overrides your object's μ.
- MuJoCo's soft contact tips boxes over well before rigid-body theory says it should.

I measured all of these against analytic answers, about 180k runs in total, and turned them into `simdiff-check`. You can run it on a model file or a PyBullet script, or add it to CI. It also estimates numerical error without ground truth by comparing dt, dt/2 and dt/4. On a chaotic arm the estimate matched the true error with rank correlation 0.999.

One finding matters for anyone comparing engines. The dt/2 check measures numerical error, but it can't see setting mistakes. With a default floor friction, the true error was 30x the estimate. That's why the linter and the self-check are separate tools.

<repo link>

Question for the thread: has anyone checked whether a policy's sim-to-real gap shrinks after fixing defaults like these? I'd like to know if it matters as much for learned policies as it does for raw physics.
