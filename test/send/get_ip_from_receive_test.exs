defmodule Send.GetIpFromReceiveTest do
  use ExUnit.Case
  require Logger

  setup do
    ViaUtils.Comms.Supervisor.start_link(nil)

    config = [
      receive: [
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
        publish_downward_tof_distance_interval_ms: 200
      ],
      send: [
        source_port: 49003,
        destination_port: 49000
      ]
    ]

    XplaneIntegration.Receive.start_link(config[:receive])
    XplaneIntegration.Send.start_link(config[:send])
    {:ok, []}
  end

  test "add subs test" do
    Process.sleep(5000)

    acts_and_outs = %{
      aileron_scaled: 0.25,
      elevator_scaled: 0.5,
      throttle_scaled: 0.75,
      rudder_scaled: -0.4,
      flaps_sdaled: 0.3
    }

    GenServer.cast(XplaneIntegration.Send, {:simulation_update_actuators, acts_and_outs, false})
    Process.sleep(500_000)
  end
end
