# Bridge Builder

A single-file browser game: build a bridge from road, wood, steel and cable, then watch real-ish physics decide whether the traffic makes it across. Open `bridge-builder.html` in any modern browser (desktop or phone). It needs no build step and no server.

## What's in it

- **16 levels**, each inspired by a real bridge (Golden Gate, Forth, Sydney Harbour, Millau, Tacoma Narrows, Akashi Kaikyō and more), with a real photo and a short engineering fact in the level briefing.
- **Level twists**: shipping lanes that must stay clear, a far bank 3 m higher, deep gorges, wind gusts that make a floppy deck gallop apart, towers standing in the water, material restrictions, and convoys of up to four vehicles at once.
- **Six vehicles** with realistic masses and axle layouts: hatchback (1.2 t), delivery van (2.2 t), box truck (4 t), dump truck (7 t, 3 axles), school bus (8 t) and semi truck (12 t, 5 axles). Each rides on suspension that bounces and feeds dynamic loads into the deck, and slows or stalls on steep grades.
- **Physics** (XPBD truss solver): members carry tension and compression, long members buckle, and cables only pull. A broken member splits into two stubs that stay pinned to their joints and dangle. Debris splashes into the water and slows down there.
- **Rendering**: themed skies (sunset, storm with rain and lightning, alpine snow peaks, city skylines at dusk), parallax mountains, textured cliffs, roads on the banks, water with live reflections, detailed beams (I-beam steel with rivets, timber grain, deck with railings), vehicles with spinning wheels, and live stress colours with a per-member force tooltip.
- **Building tools**: zoom and pan (mouse wheel, pinch, two-finger drag, or the on-screen buttons), mirror building, undo/redo, 0.5×/1×/2× simulation speed, synthesized sound effects, and per-level stars saved in the browser.

## Photos

Photos are loaded from [Wikimedia Commons](https://commons.wikimedia.org) at runtime through `Special:FilePath` links, so the game stays small and works offline (it shows a placeholder if a photo can't load). Each briefing links to the photo's file page, and the in-game **Photo credits** screen lists every source with its author and licence where known.

## Verifying the levels

`tools/test-levels.mjs` pulls the DOM-free physics block (`// PHYSICS-START` … `// PHYSICS-END`) out of the HTML. For every level it builds a reference bridge, checks that bridge against the in-game build rules, optimizes its materials, and simulates the crossing. It also checks that each level's budget and star thresholds are consistent, and that naive designs fail where a level's twist should matter: an unbraced arch, or a storm deck without a stiffening truss.

```sh
node games/bridge-builder/tools/test-levels.mjs           # all levels
node games/bridge-builder/tools/test-levels.mjs storm -v  # one level, with every member failure printed
```

Budgets are set at about 1.15–1.3× the optimized reference cost. You get 2★ for roughly matching the reference design and 3★ for beating it by about 15%.
