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

## 3. Load Identification Data

```
# This format is suitable for the following services
#   - iden_tcp_load
#   - ft_sensor_iden_load

# estimated mass(kg), only required for Lite6 models via the `iden_tcp_load` service
float32 estimated_mass 0.0

---

int16 ret
string message

# the result of identification
#   iden_tcp_load: [mass(kg)，x_centroid(mm)，y_centroid(mm)，z_centroid(mm)]
#   ft_sensor_iden_load: [mass(kg)，x_centroid(mm)，y_centroid(mm)，z_centroid(mm)，Fx_offset，Fy_offset，Fz_offset，Tx_offset，Ty_offset，Tz_ffset]
float32[] datas
```

results in service `iden_tcp_load`:

$$
datas=[ 1.920130, 0.928751, 6.010642, 93.082443 ]
$$

results in service `ft_sensor_iden_load`:

$$
datas=[1.5586555004119873, -0.7519673109054565, 5.394152641296387, 81.60640716552734, -7.101831912994385, 0.25009962916374207, 117.40576934814453, -1.0200318098068237, -0.7859300374984741, 0.8863462805747986]\\
datas=[1.5738086700439453, -0.13199445605278015, 5.315857410430908, 82.37713623046875, -6.875654220581055, 0.6403356194496155, 117.46389770507812, -0.9900681972503662, -0.7873878479003906, 0.8946043252944946]\\
datas=[1.5833412408828735, -1.378709077835083, 5.8908820152282715, 88.59040832519531, -6.836687088012695, 0.5996332764625549, 117.28087615966797, -0.9997793436050415, -0.7832141518592834, 0.8955727219581604]
$$

## 4. Gripper Force Contact

Gripper contact point:
$$
(0.0, 0.0, 200.0)
$$

## Interface

- Gripper init distance problem. (GUI or not)
- Gripper force control.