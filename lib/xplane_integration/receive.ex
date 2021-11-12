defmodule XplaneIntegration.Receive do
  require Logger
  use Bitwise
  use GenServer
  require ViaUtils.Constants, as: VC
  require ViaUtils.Shared.ValueNames, as: SVN
  @dt_accel_gyro_loop :dt_accel_gyro_loop
  @gps_pos_vel_loop :gps_pos_vel_loop
  @gps_relhdg_loop :gps_relhdg_loop
  @airspeed_loop :airspeed_loop
  @down_range_loop :down_range_loop

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

    publish_downward_range_distance_interval_ms =
      config[:publish_downward_range_distance_interval_ms]

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
      downward_range_max_m: Keyword.get(config, :downward_range_max_m, 0),
      dt_accel_gyro_group: config[:dt_accel_gyro_group],
      gps_itow_position_velocity_group: config[:gps_itow_position_velocity_group],
      gps_itow_relheading_group: config[:gps_itow_relheading_group],
      airspeed_group: config[:airspeed_group],
      downward_range_distance_group: config[:downward_range_distance_group],
      downward_range_module: config[:downward_range_module],
      publish_dt_accel_gyro_interval_ms: publish_dt_accel_gyro_interval_ms,
      publish_gps_position_velocity_interval_ms: publish_gps_position_velocity_interval_ms,
      publish_gps_relative_heading_interval_ms: publish_gps_relative_heading_interval_ms,
      publish_airspeed_interval_ms: publish_airspeed_interval_ms,
      publish_downward_range_distance_interval_ms: publish_downward_range_distance_interval_ms
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

    ViaUtils.Process.start_loop(
      self(),
      publish_downward_range_distance_interval_ms,
      @down_range_loop
    )

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
    %{bodyaccel_mpss: bodyaccel_mpss, bodyrate_rps: bodyrate_rps, dt_accel_gyro_group: group} =
      state

    unless Enum.empty?(bodyaccel_mpss) or Enum.empty?(bodyrate_rps) do
      ViaSimulation.Comms.publish_dt_accel_gyro(
        __MODULE__,
        dt_s,
        bodyaccel_mpss,
        bodyrate_rps,
        group
      )
    end

    {:noreply, state}
  end

  @impl GenServer
  def handle_info(@gps_pos_vel_loop, state) do
    %{
      position_rrm: position_rrm,
      velocity_mps: velocity_mps,
      gps_itow_position_velocity_group: group
    } = state

    unless Enum.empty?(position_rrm) or Enum.empty?(velocity_mps) do
      ViaSimulation.Comms.publish_gps_itow_position_velocity(
        __MODULE__,
        position_rrm,
        velocity_mps,
        group
      )
    end

    {:noreply, state}
  end

  @impl GenServer
  def handle_info(@gps_relhdg_loop, state) do
    %{attitude_rad: attitude_rad, gps_itow_relheading_group: group} = state

    unless Enum.empty?(attitude_rad) do
      %{SVN.yaw_rad() => yaw_rad} = attitude_rad

      ViaSimulation.Comms.publish_gps_relheading(
        __MODULE__,
        yaw_rad,
        group
      )
    end

    {:noreply, state}
  end

  @impl GenServer
  def handle_info(@airspeed_loop, state) do
    %{airspeed_mps: airspeed_mps, airspeed_group: group} = state

    unless is_nil(airspeed_mps) do
      ViaUtils.Comms.cast_global_msg_to_group(
        __MODULE__,
        {group, airspeed_mps},
        self()
      )
    end

    {:noreply, state}
  end

  @impl GenServer
  def handle_info(@down_range_loop, state) do
    %{
      attitude_rad: attitude_rad,
      agl_m: agl_m,
      downward_range_distance_group: group,
      downward_range_max_m: downward_range_max_m,
      downward_range_module: downward_range_module
    } = state

    unless Enum.empty?(attitude_rad) or is_nil(agl_m) do
      range_m =
        ViaUtils.Motion.agl_to_range_measurement(attitude_rad, agl_m, downward_range_max_m)

      if range_m < downward_range_max_m do
        # Logger.debug("pub agl/range: #{ViaUtils.Format.eftb(agl_m, 1)}/#{ViaUtils.Format.eftb(range_meas, 1)}")
        ViaSimulation.Comms.publish_downward_range_distance(
          __MODULE__,
          range_m,
          downward_range_module,
          group
        )
      end
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

              # Logger.debug("#{ViaUtils.Location.to_string(position_rrm)}/#{ViaUtils.Format.eftb(agl_m,1)}")
              %{state | position_rrm: position_rrm, agl_m: agl_m}

            21 ->
              velocity_mps = parse_velocity(buffer)
              current_time_us = :os.system_time(:microsecond)

              %{
                velocity_time_prev_us: velocity_time_prev_us,
                attitude_rad: attitude_rad,
                bodyaccel_mpss: current_bodyaccel_mpss
              } = state

              dt_s =
                if is_nil(velocity_time_prev_us),
                  do: 0.006,
                  else: (current_time_us - velocity_time_prev_us) * 1.0e-6

              {bodyaccel_mpss, velocity_time_prev_us} =
                if dt_s > 0.005 do
                  attitude_rad =
                    if Enum.empty?(attitude_rad),
                      do: %{SVN.roll_rad() => 0.0, SVN.pitch_rad() => 0.0, SVN.yaw_rad() => 0.0},
                      else: attitude_rad

                  velocity_prev_mps =
                    if Enum.empty?(velocity_mps),
                      do: %{SVN.v_north_mps() => 0, SVN.v_east_mps() => 0, SVN.v_down_mps() => 0},
                      else: velocity_mps

                  bodyaccel_mpss =
                    calculate_body_accel(velocity_mps, velocity_prev_mps, attitude_rad, dt_s)

                  # Logger.debug(
                  #   "dt/vNED:#{ViaUtils.Format.eftb(dt_s, 4)}/#{ViaUtils.Format.eftb_map(velocity_mps, 1)}"
                  # )

                  # Logger.debug("bodyaccel: #{ViaUtils.Format.eftb_map(bodyaccel_mpss, 3)}")

                  {bodyaccel_mpss, current_time_us}
                else
                  Logger.debug("XP Vel Cluster: #{dt_s}")
                  {current_bodyaccel_mpss, velocity_time_prev_us}
                end

              %{
                state
                | velocity_mps: velocity_mps,
                  bodyaccel_mpss: bodyaccel_mpss,
                  velocity_time_prev_us: velocity_time_prev_us
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

  @spec parse_airspeed(list()) :: number()
  def parse_airspeed(buffer) do
    {indicated_airspeed_knots_uint32, _buffer} = Enum.split(buffer, 4)

    ViaUtils.Enum.list_to_int_little_end(indicated_airspeed_knots_uint32)
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
          ViaUtils.Enum.list_to_int_little_end(value)
          |> ViaUtils.Math.fp_from_uint(32)

        Logger.warn(ViaUtils.Format.eftb(value, 3))
        buffer
      end)

      %{SVN.accel_x_mpss() => 0, SVN.accel_y_mpss() => 0, SVN.accel_z_mpss() => 0}
    else
      {_mach_uint32, buffer} = Enum.split(buffer, 4)
      {_vvi, buffer} = Enum.split(buffer, 4)
      {_unknown, buffer} = Enum.split(buffer, 4)
      {_unknown, buffer} = Enum.split(buffer, 4)
      {accel_z_g_uint32, buffer} = Enum.split(buffer, 4)
      {accel_x_g_uint32, buffer} = Enum.split(buffer, 4)

      {accel_y_g_uint32, _buffer} = Enum.split(buffer, 4)

      accel_z_mpss =
        ViaUtils.Enum.list_to_int_little_end(accel_z_g_uint32)
        |> ViaUtils.Math.fp_from_uint(32)
        |> Kernel.-(1)
        |> Kernel.*(-VC.gravity())

      accel_x_mpss =
        ViaUtils.Enum.list_to_int_little_end(accel_x_g_uint32)
        |> ViaUtils.Math.fp_from_uint(32)
        |> Kernel.*(VC.gravity())

      accel_y_mpss =
        ViaUtils.Enum.list_to_int_little_end(accel_y_g_uint32)
        |> ViaUtils.Math.fp_from_uint(32)
        |> Kernel.*(VC.gravity())

      %{
        SVN.accel_x_mpss() => accel_x_mpss,
        SVN.accel_y_mpss() => accel_y_mpss,
        SVN.accel_z_mpss() => accel_z_mpss
      }
    end
  end

  @spec parse_bodyrate(list()) :: map()
  def parse_bodyrate(buffer) do
    {gy_rps_uint32, buffer} = Enum.split(buffer, 4)
    {gx_rps_uint32, buffer} = Enum.split(buffer, 4)
    {gz_rps_uint32, _buffer} = Enum.split(buffer, 4)

    gx_rps =
      ViaUtils.Enum.list_to_int_little_end(gx_rps_uint32)
      |> ViaUtils.Math.fp_from_uint(32)

    gy_rps =
      ViaUtils.Enum.list_to_int_little_end(gy_rps_uint32)
      |> ViaUtils.Math.fp_from_uint(32)

    gz_rps =
      ViaUtils.Enum.list_to_int_little_end(gz_rps_uint32)
      |> ViaUtils.Math.fp_from_uint(32)

    %{SVN.gyro_x_rps() => gx_rps, SVN.gyro_y_rps() => gy_rps, SVN.gyro_z_rps() => gz_rps}
  end

  @spec parse_attitude(list()) :: map()
  def parse_attitude(buffer) do
    {pitch_deg_uint32, buffer} = Enum.split(buffer, 4)
    {roll_deg_uint32, buffer} = Enum.split(buffer, 4)
    {yaw_deg_uint32, _buffer} = Enum.split(buffer, 4)

    yaw_deg = ViaUtils.Enum.list_to_int_little_end(yaw_deg_uint32) |> ViaUtils.Math.fp_from_uint(32)

    pitch_deg =
      ViaUtils.Enum.list_to_int_little_end(pitch_deg_uint32)
      |> ViaUtils.Math.fp_from_uint(32)

    roll_deg =
      ViaUtils.Enum.list_to_int_little_end(roll_deg_uint32)
      |> ViaUtils.Math.fp_from_uint(32)

    %{
      SVN.roll_rad() => roll_deg * VC.deg2rad(),
      SVN.pitch_rad() => pitch_deg * VC.deg2rad(),
      SVN.yaw_rad() => yaw_deg * VC.deg2rad()
    }
  end

  @spec parse_position_agl(list()) :: tuple()
  def parse_position_agl(buffer) do
    {latitude_deg_uint32, buffer} = Enum.split(buffer, 4)
    {longitude_deg_uint32, buffer} = Enum.split(buffer, 4)
    {altitude_ft_uint32, buffer} = Enum.split(buffer, 4)
    {agl_ft_uint32, _buffer} = Enum.split(buffer, 4)

    latitude_deg =
      ViaUtils.Enum.list_to_int_little_end(latitude_deg_uint32)
      |> ViaUtils.Math.fp_from_uint(32)

    longitude_deg =
      ViaUtils.Enum.list_to_int_little_end(longitude_deg_uint32)
      |> ViaUtils.Math.fp_from_uint(32)

    altitude_ft =
      ViaUtils.Enum.list_to_int_little_end(altitude_ft_uint32)
      |> ViaUtils.Math.fp_from_uint(32)

    agl_ft = ViaUtils.Enum.list_to_int_little_end(agl_ft_uint32) |> ViaUtils.Math.fp_from_uint(32)

    {%{
       SVN.latitude_rad() => latitude_deg * VC.deg2rad(),
       SVN.longitude_rad() => longitude_deg * VC.deg2rad(),
       SVN.altitude_m() => altitude_ft * VC.ft2m()
     }, agl_ft * VC.ft2m()}
  end

  @spec parse_velocity(list()) :: map()
  def parse_velocity(buffer) do
    buffer = Enum.drop(buffer, 12)
    {vel_east_mps_uint32, buffer} = Enum.split(buffer, 4)
    {vel_up_mps_uint32, buffer} = Enum.split(buffer, 4)
    {vel_south_mps_uint32, _buffer} = Enum.split(buffer, 4)

    vel_north_mps =
      -(ViaUtils.Enum.list_to_int_little_end(vel_south_mps_uint32)
        |> ViaUtils.Math.fp_from_uint(32))

    vel_east_mps =
      ViaUtils.Enum.list_to_int_little_end(vel_east_mps_uint32)
      |> ViaUtils.Math.fp_from_uint(32)

    vel_down_mps =
      -(ViaUtils.Enum.list_to_int_little_end(vel_up_mps_uint32)
        |> ViaUtils.Math.fp_from_uint(32))

    %{
      SVN.v_north_mps() => vel_north_mps,
      SVN.v_east_mps() => vel_east_mps,
      SVN.v_down_mps() => vel_down_mps
    }
  end

  @spec calculate_body_accel(map(), map(), map(), number()) :: map()
  def calculate_body_accel(velocity, velocity_prev_mps, attitude_rad, dt_s) do
    %{
      SVN.v_north_mps() => v_north_mps,
      SVN.v_east_mps() => v_east_mps,
      SVN.v_down_mps() => v_down_mps
    } = velocity

    %{
      SVN.v_north_mps() => v_north_prev_mps,
      SVN.v_east_mps() => v_east_prev_mps,
      SVN.v_down_mps() => v_down_prev_mps
    } = velocity_prev_mps

    accel_inertial =
      if dt_s > 0 do
        {(v_north_mps - v_north_prev_mps) / dt_s, (v_east_mps - v_east_prev_mps) / dt_s,
         (v_down_mps - v_down_prev_mps) / dt_s - VC.gravity()}
      else
        {0, 0, -VC.gravity()}
      end

    # Logger.debug("iner: #{ViaUtils.Format.eftb_list(Tuple.to_list(accel_inertial), 3)}")
    {abx, aby, abz} = ViaUtils.Motion.inertial_to_body_euler_rad(attitude_rad, accel_inertial)

    %{
      SVN.accel_x_mpss() => abx,
      SVN.accel_y_mpss() => aby,
      SVN.accel_z_mpss() => abz
    }
  end
end
