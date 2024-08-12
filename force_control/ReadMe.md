# Force Control Demo

## 1. Services

Replace `false` with `true` in the following sercives of `xarm_api/config/xarm_params.yaml`.

```yaml
ft_sensor_app_get: true
get_ft_sensor_error: true
ft_sensor_enable: true
ft_sensor_app_set: true
get_ft_sensor_data: true
ft_sensor_iden_load: true
ft_sensor_cali_load: true
config_force_control: true

set_force_control_pid: true # it was lost in the formal version
```

Also remains following services:

```yaml
save_record_trajectory: false
load_trajectory: false
```

## 2. Config

In service `/xarm/config_force_control`, params list:

```srv
int16 coord
int16[] c_axis
float32[] ref
float32[] limits

---

int16 ret
string message
```

In service `/xarm/set_force_control_pid`, params list:

```srv
# This format is suitable for the following services
#   - set_force_control_pid

float32[] kp
float32[] ki
float32[] kd
float32[] xe_limit

---

int16 ret
string message
```

First to set the params of force control and enable ft sensor with enable 1.
Then, set sensor app in service `ft_sensor_app_set`, set app 2 for force control. Function will activate after state set 0. Set app 0 and ft sensor 0 when finished.
