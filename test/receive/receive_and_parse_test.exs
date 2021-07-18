defmodule Receive.ReceiveAndParseTest do
  use ExUnit.Case
  require Logger

  setup do
    ViaUtils.Comms.Supervisor.start_link(nil)
    config = [
      port: 49002,
      dt_accel_gyro_group: :dt_accel_gyro_val,
      gps_itow_position_velocity_group: :gps_itow_position_velocity_val,
      gps_itow_relheading_group: :gps_itow_relheading_group,
      airspeed_group: :airspeed_val,
      downward_tof_distance_group: :downward_tof_distance_val,
      publish_dt_accel_gyro_interval_ms: 5,
      publish_gps_position_velocity_interval_ms: 200,
      publish_gps_relative_heading_interval_ms: 200,
      publish_airspeed_interval_ms: 200,
      publish_downward_tof_distance_interval_ms: 200,

    ]
    XplaneIntegration.Receive.start_link(config)
    {:ok, []}
  end

  test "add subs test" do
    Process.sleep(10000)
  end
end
