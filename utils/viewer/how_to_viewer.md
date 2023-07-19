



## QUAD 2d Recovery with Obstacles


check which primitives are applicable
m4 test_db && ./test_db --run_test=t_plot_primitives

extract primitves
make test_croco  && ./test_croco --run_test=t_extract_primitives

python3 ../viewer/viewer_cli.py --robot quad2d   --env ../benchmark/quad2d/quad2d_recovery_obs.yaml
   --result ../results_new/quad2d/quad2d_recovery_obs/idbastar_v0/2023-06-21--12-40-54/run_0_out.ya
ml.trajopt-0.yaml

python3 ../viewer/viewer_cli.py --robot quad2d   --env ../benchmark/quad2d/quad2d_recovery_obs.yaml
   --result ../results_new/quad2d/quad2d_recovery_obs/idbastar_v0/2023-06-21--12-40-54/run_0_out.ya
ml.trajraw-0.yaml


## COMPARE the three algorithm

(croco) ⋊> ~/s/w/k/build on dev ⨯ python3 ../viewer/viewer_cli.py --robot unicycle2   --env ../benchmark/unicycle_second_order_0/bugtrap_0.yaml   --result ../results_new/unicycle_second_order_0/bugtrap_0
//sst_v0/2023-06-20--19-31-06/run_0_out.yaml.trajraw-0.yaml

(croco) ⋊> ~/s/w/k/build on dev ⨯ python3 ../viewer/viewer_cli.py --robot unicycle2   --env ../benchmark/unicycle_second_order_0/bugtrap_0.yaml   --result ../results_new/unicycle_second_order_0/bugtrap_0
/idbastar_v0/2023-06-21--12-38-34/run_0_out.yaml.trajraw-0.yaml --result2 ../results_new/unicycle_second_order_0/bugtrap_0/idbastar_v0/2023-06-21--12-38-34/run_0_out.yaml.trajopt-0.yaml

(croco) ⋊> ~/s/w/k/build on dev ⨯ python3 ../viewer/viewer_cli.py --robot unicycle2   --env ../benchmark/unicycle_second_order_0/bugtrap_0.yaml   --result ../results_new/unicycle_second_order_0/bugtrap_0
//geo_v1/2023-06-20--19-41-53/run_1_out.yaml.trajraw-3.yaml --result2 ../results_new/unicycle_second_order_0/bugtrap_0/geo_v1/2023-06-20--19-41-53/run_1_out.yaml.trajopt-3.yaml

