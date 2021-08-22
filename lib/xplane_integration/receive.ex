defmodule XplaneIntegration.Receive do
  require Logger
  use Bitwise
  use GenServer
  require ViaUtils.Constants, as: VC

  @dt_accel_gyro_loop :dt_accel_gyro_loop
  @gps_pos_vel_loop :gps_pos_vel_loop
  @gps_relhdg_loop :gps_relhdg_loop
  @airspeed_loop :airspeed_loop
  @down_tof_loop :down_tof_loop

  def start_link(config) do
    Logger.debug("Start Simulation.XplaneReceive")
    ViaUtils.Process.start_link_redundant(GenServer, __MODULE__, config, __MODULE__)
  end

  @impl GenServer
  def init(config) do
    port = Keyword.fetch!(config, :port)
    {:ok, socket} = :gen_udp.open(port, broadcast: false, active: true)

    publish_dt_accel_gyro_interval_ms = config[:publish_dt_accel_gyro_interval_ms]
    publish_gps_position_velocity_interval_ms = config[:publish_gps_position_velocity_interval_ms]
    publish_gps_relative_heading_interval_ms = config[:publish_gps_relative_heading_interval_ms]
    publish_airspeed_interval_ms = config[:publish_airspeed_interval_ms]
    publish_downward_tof_distance_interval_ms = config[:publish_downward_tof_distance_interval_ms]

    state = %{
      socket: socket,
      port: port,
      source_ip_address: nil,
      bodyaccel_mpss: %{},
      attitude_rad: %{},
      bodyrate_rps: %{},
      position_rrm: %{},
      velocity_mps: %{},
      agl_m: nil,
      airspeed_mps: nil,
      velocity_time_prev_us: nil,
      dt_accel_gyro_group: config[:dt_accel_gyro_group],
      gps_itow_position_velocity_group: config[:gps_itow_position_velocity_group],
      gps_itow_relheading_group: config[:gps_itow_relheading_group],
      airspeed_group: config[:airspeed_group],
      downward_tof_distance_group: config[:downward_tof_distance_group],
      publish_dt_accel_gyro_interval_ms: publish_dt_accel_gyro_interval_ms,
      publish_gps_position_velocity_interval_ms: publish_gps_position_velocity_interval_ms,
      publish_gps_relative_heading_interval_ms: publish_gps_relative_heading_interval_ms,
      publish_airspeed_interval_ms: publish_airspeed_interval_ms,
      publish_downward_tof_distance_interval_ms: publish_downward_tof_distance_interval_ms
    }

    ViaUtils.Comms.Supervisor.start_operator(__MODULE__)

    ViaUtils.Process.start_loop(
      self(),
      publish_dt_accel_gyro_interval_ms,
      {@dt_accel_gyro_loop, publish_dt_accel_gyro_interval_ms * 1.0e-3}
    )

    ViaUtils.Process.start_loop(
      self(),
      publish_gps_position_velocity_interval_ms,
      @gps_pos_vel_loop
    )

    ViaUtils.Process.start_loop(
      self(),
      publish_gps_relative_heading_interval_ms,
      @gps_relhdg_loop
    )

    ViaUtils.Process.start_loop(self(), publish_downward_tof_distance_interval_ms, @down_tof_loop)
    {:ok, state}
  end

  @impl GenServer
  def handle_cast({:get_ip_address, from}, state) do
    src_ip = state.source_ip_address

    unless is_nil(src_ip) do
      GenServer.cast(from, {:set_ip_address, src_ip})
    end
    {:noreply, state}
  end

  @impl GenServer
  def handle_info({:udp, _socket, src_ip, _src_port, msg}, state) do
    # Logger.debug("received data from #{inspect(src_ip)} on port #{src_port} with length #{length(msg)}")
    state =
      parse_data_buffer(msg, state)
      |> Map.put(:source_ip_address, src_ip)

    {:noreply, state}
  end

  @impl GenServer
  def handle_info({@dt_accel_gyro_loop, dt_s}, state) do
    bodyaccel_mpss = state.bodyaccel_mpss
    bodyrate_rps = state.bodyrate_rps

    unless Enum.empty?(bodyaccel_mpss) or Enum.empty?(bodyrate_rps) do
      publish_dt_accel_gyro(
        dt_s,
        bodyaccel_mpss,
        bodyrate_rps,
        state.dt_accel_gyro_group
      )
    end

    {:noreply, state}
  end

  @impl GenServer
  def handle_info(@gps_pos_vel_loop, state) do
    position_rrm = state.position_rrm
    velocity_mps = state.velocity_mps

    unless Enum.empty?(position_rrm) or Enum.empty?(velocity_mps) do
      publish_gps_itow_position_velocity(
        position_rrm,
        velocity_mps,
        state.gps_itow_position_velocity_group
      )
    end

    {:noreply, state}
  end

  @impl GenServer
  def handle_info(@gps_relhdg_loop, state) do
    attitude_rad = state.attitude_rad

    unless Enum.empty?(attitude_rad) do
      publish_gps_relheading(attitude_rad.yaw_rad, state.gps_itow_relheading_group)
    end

    {:noreply, state}
  end

  @impl GenServer
  def handle_info(@airspeed_loop, state) do
    airspeed_mps = state.airspeed_mps

    unless is_nil(airspeed_mps) do
      publish_airspeed(airspeed_mps, state.airspeed_group)
    end

    {:noreply, state}
  end

  @impl GenServer
  def handle_info(@down_tof_loop, state) do
    attitude_rad = state.attitude_rad
    agl_m = state.agl_m

    unless Enum.empty?(attitude_rad) or is_nil(agl_m) do
      publish_downward_tof_distance(attitude_rad, agl_m, state.downward_tof_distance_group)
    end

    {:noreply, state}
  end

  @spec parse_data_buffer(list(), map()) :: map()
  def parse_data_buffer(entire_buffer, state) do
    {header, data_buffer} = Enum.split(entire_buffer, 4)

    if header == [68, 65, 84, 65] do
      data_buffer = Enum.drop(data_buffer, 1)
      parse_message(data_buffer, state)
    else
      state
    end
  end

  @spec parse_message(list(), map()) :: map()
  def parse_message(buffer, state) do
    message_type = Enum.at(buffer, 0)
    buffer = Enum.drop(buffer, 4)
    # Logger.debug("msg: #{message_type}")
    {state, buffer} =
      if length(buffer) >= 32 do
        state =
          case message_type do
            3 ->
              indicated_airspeed_mps = parse_airspeed(buffer)
              %{state | airspeed_mps: indicated_airspeed_mps}

            # 4 ->
            #   # Add accel due to gravity
            #   # Logger.debug("accel_mpss xyz: #{eftb(accel_x_mpss,3)}/#{eftb(accel_y_mpss, 3)}/#{eftb(accel_z_mpss, 3)}")
            #   attitude =
            #     if Enum.empty?(state.attitude_rad),
            #       do: %{roll_rad: 0.0, pitch_rad: 0.0, yaw_rad: 0.0},
            #       else: state.attitude_rad

            #   accel_gravity = ViaUtils.Motion.attitude_to_accel_rad(attitude)

            #   accel_inertial = parse_accel_inertial(buffer)
            #   # Logger.debug("accel i xyz: #{ViaUtils.Format.eftb_map(accel_inertial, 3)}")
            #   # Logger.debug("accel g xyz: #{ViaUtils.Format.eftb_map(accel_gravity, 3)}")

            #   accel = %{
            #     ax_mpss: accel_gravity.x + accel_inertial.ax_mpss,
            #     ay_mpss: accel_gravity.y + accel_inertial.ay_mpss,
            #     az_mpss: accel_gravity.z + accel_inertial.az_mpss
            #   }

            #   # Logger.debug("accel xyz: #{ViaUtils.Format.eftb_map(accel,3)}")
            #   %{state | bodyaccel_mpss: accel}

            16 ->
              bodyrate_rps = parse_bodyrate(buffer)

              # Logger.debug("body: #{ViaUtils.Format.eftb_map_deg(bodyrate_rps, 1)}")
              %{state | bodyrate_rps: bodyrate_rps}

            17 ->
              attitude_rad = parse_attitude(buffer)
              # Logger.debug("XP rpy: #{ViaUtils.Format.eftb_map_deg(attitude_rad, 1)}")
              %{state | attitude_rad: attitude_rad}

            20 ->
              {position_rrm, agl_m} = parse_position_agl(buffer)

              # Logger.debug("lat/lon/alt/agl: #{ViaUtils.Location.to_string(position_rrm)}/#{ViaUtils.Format.eftb(agl_m,1)}")
              %{state | position_rrm: position_rrm, agl_m: agl_m}

            21 ->
              velocity_mps = parse_velocity(buffer)
              current_time_us = :os.system_time(:microsecond)

              dt_s =
                if is_nil(state.velocity_time_prev_us),
                  do: 0,
                  else: (current_time_us - state.velocity_time_prev_us) * 1.0e-6

              attitude_rad =
                if Enum.empty?(state.attitude_rad),
                  do: %{roll_rad: 0.0, pitch_rad: 0.0, yaw_rad: 0.0},
                  else: state.attitude_rad

              velocity_prev_mps =
                if Enum.empty?(state.velocity_mps),
                  do: %{north_mps: 0, east_mps: 0, down_mps: 0},
                  else: state.velocity_mps

              bodyaccel_mpss =
                calculate_body_accel(velocity_mps, velocity_prev_mps, attitude_rad, dt_s)

              # Logger.debug("vNED: #{ViaUtils.Format.eftb_map(velocity_mps, 1)}")
              # Logger.debug("bodyaccel: #{ViaUtils.Format.eftb_map(bodyaccel_mpss, 3)}")
              %{
                state
                | velocity_mps: velocity_mps,
                  bodyaccel_mpss: bodyaccel_mpss,
                  velocity_time_prev_us: current_time_us
              }

            other ->
              Logger.debug("unknown type: #{other}")
              state
          end

        {state, Enum.drop(buffer, 32)}
      else
        {state, []}
      end

    # If there is more data in the buffer, parse it
    unless Enum.empty?(buffer) do
      parse_message(buffer, state)
    else
      state
    end
  end

  @spec publish_gps_itow_position_velocity(map(), map(), any()) :: atom()
  def publish_gps_itow_position_velocity(position_rrm, velocity_mps, group) do
    # TODO: Get correct value for itow_ms
    itow_ms = nil

    # Logger.debug(
    #   "pub gps pos/vel: #{ViaUtils.Location.to_string(position_rrm)}/#{ViaUtils.Format.eftb_map(velocity_mps, 1)}"
    # )

    ViaUtils.Comms.send_global_msg_to_group(
      __MODULE__,
      {group, itow_ms, position_rrm, velocity_mps},
      self()
    )
  end

  @spec publish_gps_relheading(float(), any()) :: atom()
  def publish_gps_relheading(rel_heading_rad, group) do
    # TODO: Get correct value for itow_ms
    itow_ms = nil

    # Logger.debug("pub relhdg: #{ViaUtils.Format.eftb_deg(rel_heading_rad, 1)}")

    ViaUtils.Comms.send_global_msg_to_group(
      __MODULE__,
      {group, itow_ms, rel_heading_rad},
      self()
    )
  end

  @spec publish_dt_accel_gyro(float(), map(), map(), any()) :: atom()
  def publish_dt_accel_gyro(dt_s, accel_mpss, gyro_rps, group) do
    values =
      %{dt_s: dt_s}
      |> Map.merge(accel_mpss)
      |> Map.merge(gyro_rps)

    # Logger.debug("pub dtaccgy: #{ViaUtils.Format.eftb_map(values,4)}")
    ViaUtils.Comms.send_global_msg_to_group(
      __MODULE__,
      {group, values},
      self()
    )
  end

  @spec publish_airspeed(float(), any()) :: atom()
  def publish_airspeed(airspeed_mps, group) do
    Logger.debug("pub A/S: #{ViaUtils.Format.eftb(airspeed_mps, 1)}")

    ViaUtils.Comms.send_global_msg_to_group(
      __MODULE__,
      {group, airspeed_mps},
      self()
    )
  end

  @spec publish_downward_tof_distance(map(), float(), any()) :: atom()
  def publish_downward_tof_distance(attitude_rad, agl_m, group) do
    range_meas = agl_m / (:math.cos(attitude_rad.roll_rad) * :math.cos(attitude_rad.pitch_rad))
    range_meas = if range_meas < 0, do: 0, else: range_meas

    # Logger.debug("pub tof: #{ViaUtils.Format.eftb(range_meas, 1)}")

    ViaUtils.Comms.send_global_msg_to_group(
      __MODULE__,
      {group, range_meas},
      self()
    )
  end

  @spec parse_airspeed(list()) :: number()
  def parse_airspeed(buffer) do
    {indicated_airspeed_knots_uint32, _buffer} = Enum.split(buffer, 4)

    ViaUtils.Enum.list_to_int(indicated_airspeed_knots_uint32, 4)
    |> ViaUtils.Math.fp_from_uint(32)
    |> Kernel.*(VC.knots2mps())
  end

  @spec parse_accel_inertial(list()) :: map()
  def parse_accel_inertial(buffer) do
    spoof = false

    if spoof do
      Enum.reduce(1..8, buffer, fn _x, acc ->
        {value, buffer} = Enum.split(acc, 4)

        value =
          ViaUtils.Enum.list_to_int(value, 4)
          |> ViaUtils.Math.fp_from_uint(32)

        Logger.warn(ViaUtils.Format.eftb(value, 3))
        buffer
      end)

      %{ax_mpss: 0, ay_mpss: 0, az_mpss: 0}
    else
      {_mach_uint32, buffer} = Enum.split(buffer, 4)
      {_vvi, buffer} = Enum.split(buffer, 4)
      {_unknown, buffer} = Enum.split(buffer, 4)
      {_unknown, buffer} = Enum.split(buffer, 4)
      {accel_z_g_uint32, buffer} = Enum.split(buffer, 4)
      {accel_x_g_uint32, buffer} = Enum.split(buffer, 4)

      {accel_y_g_uint32, _buffer} = Enum.split(buffer, 4)

      accel_z_mpss =
        ViaUtils.Enum.list_to_int(accel_z_g_uint32, 4)
        |> ViaUtils.Math.fp_from_uint(32)
        |> Kernel.-(1)
        |> Kernel.*(-VC.gravity())

      accel_x_mpss =
        ViaUtils.Enum.list_to_int(accel_x_g_uint32, 4)
        |> ViaUtils.Math.fp_from_uint(32)
        |> Kernel.*(VC.gravity())

      accel_y_mpss =
        ViaUtils.Enum.list_to_int(accel_y_g_uint32, 4)
        |> ViaUtils.Math.fp_from_uint(32)
        |> Kernel.*(VC.gravity())

      %{ax_mpss: accel_x_mpss, ay_mpss: accel_y_mpss, az_mpss: accel_z_mpss}
    end
  end

  @spec parse_bodyrate(list()) :: map()
  def parse_bodyrate(buffer) do
    {gy_rps_uint32, buffer} = Enum.split(buffer, 4)
    {gx_rps_uint32, buffer} = Enum.split(buffer, 4)
    {gz_rps_uint32, _buffer} = Enum.split(buffer, 4)

    gx_rps =
      ViaUtils.Enum.list_to_int(gx_rps_uint32, 4)
      |> ViaUtils.Math.fp_from_uint(32)

    gy_rps =
      ViaUtils.Enum.list_to_int(gy_rps_uint32, 4)
      |> ViaUtils.Math.fp_from_uint(32)

    gz_rps =
      ViaUtils.Enum.list_to_int(gz_rps_uint32, 4)
      |> ViaUtils.Math.fp_from_uint(32)

    %{gx_rps: gx_rps, gy_rps: gy_rps, gz_rps: gz_rps}
  end

  @spec parse_attitude(list()) :: map()
  def parse_attitude(buffer) do
    {pitch_deg_uint32, buffer} = Enum.split(buffer, 4)
    {roll_deg_uint32, buffer} = Enum.split(buffer, 4)
    {yaw_deg_uint32, _buffer} = Enum.split(buffer, 4)

    yaw_deg = ViaUtils.Enum.list_to_int(yaw_deg_uint32, 4) |> ViaUtils.Math.fp_from_uint(32)

    pitch_deg =
      ViaUtils.Enum.list_to_int(pitch_deg_uint32, 4)
      |> ViaUtils.Math.fp_from_uint(32)

    roll_deg =
      ViaUtils.Enum.list_to_int(roll_deg_uint32, 4)
      |> ViaUtils.Math.fp_from_uint(32)

    %{
      roll_rad: roll_deg * VC.deg2rad(),
      pitch_rad: pitch_deg * VC.deg2rad(),
      yaw_rad: yaw_deg * VC.deg2rad()
    }
  end

  @spec parse_position_agl(list()) :: tuple()
  def parse_position_agl(buffer) do
    {latitude_deg_uint32, buffer} = Enum.split(buffer, 4)
    {longitude_deg_uint32, buffer} = Enum.split(buffer, 4)
    {altitude_ft_uint32, buffer} = Enum.split(buffer, 4)
    {agl_ft_uint32, _buffer} = Enum.split(buffer, 4)

    latitude_deg =
      ViaUtils.Enum.list_to_int(latitude_deg_uint32, 4)
      |> ViaUtils.Math.fp_from_uint(32)

    longitude_deg =
      ViaUtils.Enum.list_to_int(longitude_deg_uint32, 4)
      |> ViaUtils.Math.fp_from_uint(32)

    altitude_ft =
      ViaUtils.Enum.list_to_int(altitude_ft_uint32, 4)
      |> ViaUtils.Math.fp_from_uint(32)

    agl_ft = ViaUtils.Enum.list_to_int(agl_ft_uint32, 4) |> ViaUtils.Math.fp_from_uint(32)

    {%{
       latitude_rad: latitude_deg * VC.deg2rad(),
       longitude_rad: longitude_deg * VC.deg2rad(),
       altitude_m: altitude_ft * VC.ft2m()
     }, agl_ft * VC.ft2m()}
  end

  @spec parse_velocity(list()) :: map()
  def parse_velocity(buffer) do
    buffer = Enum.drop(buffer, 12)
    {vel_east_mps_uint32, buffer} = Enum.split(buffer, 4)
    {vel_up_mps_uint32, buffer} = Enum.split(buffer, 4)
    {vel_south_mps_uint32, _buffer} = Enum.split(buffer, 4)

    vel_north_mps =
      -(ViaUtils.Enum.list_to_int(vel_south_mps_uint32, 4)
        |> ViaUtils.Math.fp_from_uint(32))

    vel_east_mps =
      ViaUtils.Enum.list_to_int(vel_east_mps_uint32, 4)
      |> ViaUtils.Math.fp_from_uint(32)

    vel_down_mps =
      -(ViaUtils.Enum.list_to_int(vel_up_mps_uint32, 4)
        |> ViaUtils.Math.fp_from_uint(32))

    %{
      north_mps: vel_north_mps,
      east_mps: vel_east_mps,
      down_mps: vel_down_mps
    }
  end

  @spec calculate_body_accel(map(), map(), map(), number()) :: map()
  def calculate_body_accel(velocity_mps, velocity_prev_mps, attitude_rad, dt_s) do
    accel_inertial =
      if dt_s > 0 do
        {(velocity_mps.north_mps - velocity_prev_mps.north_mps) / dt_s,
         (velocity_mps.east_mps - velocity_prev_mps.east_mps) / dt_s,
         (velocity_mps.down_mps - velocity_prev_mps.down_mps) / dt_s - VC.gravity()}
      else
        {0, 0, -VC.gravity()}
      end

    # Logger.debug("iner: #{ViaUtils.Format.eftb_list(Tuple.to_list(accel_inertial), 3)}")
    {abx, aby, abz} = ViaUtils.Motion.inertial_to_body_euler_rad(attitude_rad, accel_inertial)
    # Add gravity
    # Rotate entire vector to body frame
    # Logger.debug("iner: #{ViaUtils.Format.eftb_map(%{ax: abx, ay: aby, az: abz}, 3)}")
    # Logger.debug("dt: #{ViaUtils.Format.eftb(dt_s,4)}")
    # accel_gravity = ViaUtils.Motion.attitude_to_accel_rad(attitude_rad)

    # Logger.debug("grav: #{ViaUtils.Format.eftb_map(accel_gravity, 3)}")
    %{
      ax_mpss: abx,
      ay_mpss: aby,
      az_mpss: abz
    }
  end
end
