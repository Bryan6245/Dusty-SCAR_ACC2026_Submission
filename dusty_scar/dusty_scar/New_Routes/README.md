New_Routes structure:
- shared/: scripts/utils/config used across all routes
- Route1..Route8/: each route has its own bags, extracted data, combo set, models, notes
Goal: train a separate BC model per route, then interconnect routes using a route selector / supervisor.
