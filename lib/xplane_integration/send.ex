defmodule XplaneIntegration.Send do
  use Bitwise
  use GenServer
  require Logger
  require ViaUtils.Shared.Groups, as: Groups
  require ViaUtils.Ubx.ClassDefs, as: ClassDefs
  require ViaUtils.Ubx.AccelGyro.DtAccelGyro, as: DtAccelGyro
  require ViaUtils.Ubx.VehicleCmds.BodyrateThrustCmd, as: BodyrateThrustCmd
  require ViaUtils.Ubx.VehicleCmds.ActuatorCmdDirect, as: ActuatorCmdDirect
  require ViaUtils.Ubx.VehicleCmds.BodyrateActuatorOutput, as: BodyrateActuatorOutput
  require ViaUtils.Shared.ActuatorNames, as: Act
  @cmd_header <<68, 65, 84, 65, 0>>
  @zeros_1 <<0, 0, 0, 0>>
  @zeros_3 <<0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0>>
  # @zeros_4 <<0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0>>
  @zeros_5 <<0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0>>
  @zeros_7 <<0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0>>

  @request_ip_address_loop :request_ip_address_loop
  @request_ip_address_loop_interval 1000

  def start_link(config) do
    Logger.debug("Start Simulation.XplaneSend")
    ViaUtils.Process.start_link_redundant(GenServer, __MODULE__, config, __MODULE__)
  end

  @impl GenServer
  def init(config) do
    {:ok, socket} =
      :gen_udp.open(Keyword.fetch!(config, :source_port), broadcast: false, active: true)

    request_ip_address_timer =
      ViaUtils.Process.start_loop(
        self(),
        @request_ip_address_loop_interval,
        @request_ip_address_loop
      )

    state = %{
      socket: socket,
      destination_ip_address: nil,
      destination_port: Keyword.fetch!(config, :destination_port),
      ubx: UbxInterpreter.new(),
      bodyaccel: %{},
      attitude: %{},
      bodyrate: %{},
      position: %{},
      velocity: %{},
      agl: 0,
      airspeed: 0,
      new_simulation_data_to_publish: false,
      request_ip_address_timer: request_ip_address_timer
    }

    ViaUtils.Comms.Supervisor.start_operator(__MODULE__)
    ViaUtils.Comms.join_group(__MODULE__, Groups.simulation_update_actuators())
    ViaUtils.Comms.join_group(__MODULE__, Groups.virtual_uart_actuator_output())

    {:ok, state}
  end

  @impl GenServer
  def handle_info(@request_ip_address_loop, state) do
    state =
      if is_nil(state.destination_ip_address) do
        GenServer.cast(XplaneIntegration.Receive, {:get_ip_address, self()})
        state
      else
        ViaUtils.Process.stop_loop(state.request_ip_address_timer)
        %{state | request_ip_address_timer: nil}
      end

    {:noreply, state}
  end

  @impl GenServer
  def handle_info({:circuits_uart, _port, data}, state) do
    Logger.debug("xpi rx: #{data}")
    {:noreply, state}
  end

  @impl GenServer
  def handle_cast({:set_ip_address, ip_address}, state) do
    {:noreply, %{state | destination_ip_address: ip_address}}
  end

  @spec check_for_new_messages_and_process(list(), map()) :: map()
  def check_for_new_messages_and_process(data, state) do
    %{ubx: ubx, destination_ip: dest_ip, socket: socket, destination_port: dest_port} = state
    {ubx, payload} = UbxInterpreter.check_for_new_message(ubx, data)

    if Enum.empty?(payload) do
      state
    else
      %{msg_class: msg_class, msg_id: msg_id} = ubx
      # Logger.debug("msg class/id: #{msg_class}/#{msg_id}")
      state =
        case msg_class do
          ClassDefs.vehicle_cmds() ->
            case msg_id do
              BodyrateActuatorOutput.id() ->
                cmds =
                  UbxInterpreter.deconstruct_message_to_map(
                    BodyrateActuatorOutput.bytes(),
                    BodyrateActuatorOutput.multipliers(),
                    BodyrateActuatorOutput.keys(),
                    payload
                  )

                %{
                  Act.aileron() => aileron_scaled,
                  Act.elevator() => elevator_scaled,
                  Act.throttle() => throttle_scaled,
                  Act.rudder() => rudder_scaled
                } = cmds

                throttle_scaled = ViaUtils.Math.get_one_sided_from_two_sided(throttle_scaled)

                send_ail_elev_rud_commands(
                  aileron_scaled,
                  elevator_scaled,
                  rudder_scaled,
                  socket,
                  dest_ip,
                  dest_port
                )

                send_throttle_command(throttle_scaled, socket, dest_ip, dest_port)

                # Logger.debug("dt/accel/gyro values: #{inspect([dt, ax, ay, az, gx, gy, gz])}")
                # Logger.debug("send dt/accel/gyro values: #{ViaUtils.Format.eftb_map(values, 3)}")

                %{state | ubx: UbxInterpreter.clear(ubx)}

            ActuatorCmdDirect.id ->
cmds =
                  UbxInterpreter.deconstruct_message_to_map(
                    ActuatorCmdDirect.bytes(),
                    ActuatorCmdDirect.multipliers(),
                    ActuatorCmdDirect.keys(),
                    payload
                  )

                %{
                  Act.aileron() => aileron_scaled,
                  Act.elevator() => elevator_scaled,
                  Act.throttle() => throttle_scaled,
                  Act.rudder() => rudder_scaled
                } = cmds

              _other ->
                Logger.warn("Bad message id: #{msg_id}")
                state
            end

          ViaUtils.Ubx.ClassDefs.vehicle_cmds() ->
            case msg_id do
              BodyrateThrustCmd.id() ->
                TestHelper.Companion.Utils.display_bodyrate_thrust_cmd(payload)
            end

            %{state | ubx: UbxInterpreter.clear(ubx)}

          _other ->
            Logger.warn("Bad message class: #{msg_class}")
            state
        end

      check_for_new_messages_and_process([], state)
    end
  end

  def get_cmds_from_ubx_message(msg_class, msg_id, payload) do
  end

  def send_cmds(cmds, socket, dest_ip, dest_port) do
    # Logger.debug("xp send rx up_act: #{ViaUtils.Format.eftb_map(actuators_and_outputs, 3)}")

    dest_ip = state.destination_ip_address

    unless is_nil(dest_ip) do
      cmds =
        Enum.reduce(actuators_and_outputs, %{}, fn {actuator_name, output}, acc ->
          case actuator_name do
            :flaps_scaled ->
              Map.put(acc, actuator_name, ViaUtils.Math.get_one_sided_from_two_sided(output))

            :gear_scaled ->
              Map.put(acc, actuator_name, ViaUtils.Math.get_one_sided_from_two_sided(output))

            :throttle_scaled ->
              Map.put(acc, actuator_name, ViaUtils.Math.get_one_sided_from_two_sided(output))

            name ->
              Map.put(acc, name, output)
          end
        end)

      socket = state.socket
      dest_port = state.destination_port
      send_ail_elev_rud_commands(cmds, socket, dest_ip, dest_port)
      send_throttle_command(cmds, socket, dest_ip, dest_port)
      send_flaps_command(cmds, socket, dest_ip, dest_port)
    end

    # Logger.debug("up act cmds: #{ViaUtils.Format.eftb_map(cmds, 3)}")
    {:noreply, state}
  end

  @impl GenServer
  def handle_cast({:send_commands, command_type, commands}, state) do
    function =
      case command_type do
        :ail_elev_rud -> :send_ail_elev_rud_commands
        :throttle -> :send_throttle_command
        :flaps -> :send_flaps_command
      end

    apply(__MODULE__, function, [
      commands,
      state.socket,
      state.destination_ip_address,
      state.destination_port
    ])

    {:noreply, state}
  end

  @spec send_ail_elev_rud_commands(number(), number(), number(), any(), tuple(), integer()) ::
          atom()
  def send_ail_elev_rud_commands(
        aileron_scaled,
        elevator_scaled,
        rudder_scaled,
        socket,
        dest_ip,
        port
      ) do
    buffer =
      (@cmd_header <> <<11, 0, 0, 0>>)
      |> Kernel.<>(ViaUtils.Math.uint_from_fp(elevator_scaled, 32))
      |> Kernel.<>(ViaUtils.Math.uint_from_fp(aileron_scaled, 32))
      |> Kernel.<>(ViaUtils.Math.uint_from_fp(rudder_scaled, 32))
      |> Kernel.<>(@zeros_5)

    # Logger.debug("ail/elev/rud: #{Map.get(commands, :aileron)}/#{Map.get(commands, :elevator)}/#{Map.get(commands, :rudder)}")
    # Logger.debug("ail-elev-rud: #{ViaUtils.Format.eftb_map(commands, 3)}")
    :gen_udp.send(socket, dest_ip, port, buffer)
  end

  @spec send_throttle_command(number(), any(), tuple(), integer()) :: atom()
  def send_throttle_command(throttle_scaled, socket, dest_ip, port) do
    buffer =
      (@cmd_header <> <<25, 0, 0, 0>>)
      |> Kernel.<>(ViaUtils.Math.uint_from_fp(throttle_scaled, 32))
      |> Kernel.<>(@zeros_7)

    # |> Kernel.<>(<<0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0>>)
    # Logger.debug("buffer: #{buffer}")
    # Logger.debug("thr: #{Map.get(commands, :throttle_scaled)}")
    :gen_udp.send(socket, dest_ip, port, buffer)
  end

  @spec send_flaps_command(number(), any(), tuple(), integer()) :: atom()
  def send_flaps_command(flaps_scaled, socket, dest_ip, port) do
    # Logger.error("flaps: #{Map.get(commands, :flaps_scaled)}")
    buffer =
      (@cmd_header <> <<13, 0, 0, 0>>)
      |> Kernel.<>(@zeros_3)
      |> Kernel.<>(ViaUtils.Math.uint_from_fp(flaps_scaled, 32))
      |> Kernel.<>(ViaUtils.Math.uint_from_fp(-999, 32))
      |> Kernel.<>(ViaUtils.Math.uint_from_fp(-999, 32))
      |> Kernel.<>(@zeros_1)
      |> Kernel.<>(ViaUtils.Math.uint_from_fp(-999, 32))

    :gen_udp.send(socket, dest_ip, port, buffer)
  end

  @spec send_ail_elev_rud_commands_test(map()) :: atom()
  def send_ail_elev_rud_commands_test(commands) do
    %{
      Act.aileron() => aileron_scaled,
      Act.elevator() => elevator_scaled,
      Act.rudder() => rudder_scaled
    } = commands

    GenServer.cast(
      __MODULE__,
      {:send_commands, :ail_elev_rud, aileron_scaled, elevator_scaled, rudder_scaled}
    )
  end

  @spec send_throttle_command_test(number()) :: atom()
  def send_throttle_command_test(throttle_scaled) do
    GenServer.cast(__MODULE__, {:send_commands, :throttle, throttle_scaled})
  end

  @spec send_flaps_command_test(number()) :: atom()
  def send_flaps_command_test(flaps_scaled) do
    GenServer.cast(__MODULE__, {:send_commands, :flaps, flaps_scaled})
  end

  # @spec get_one_sided_value(number()) :: number()
  # def get_one_sided_value(two_sided_value) do
  #   0.5 * two_sided_value + 0.5
  # end
end
