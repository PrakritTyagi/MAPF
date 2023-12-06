./build/lifelong --inputFile ./example_problems/warehouse.domain/warehouse_small_10.json -o PlanViz/solutions/warehouse_small_10.json

python PlanViz/script/plan_viz.py --map example_problems/warehouse.domain/maps/warehouse_small.map --plan PlanViz/solutions/warehouse_small_10.json --grid --aid --static --ca --end 4999
