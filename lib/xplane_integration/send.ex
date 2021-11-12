defmodule XplaneIntegration.Send do
  use Bitwise
  use GenServer
  require Logger
  require ViaUtils.Shared.Groups, as: Groups
  require ViaTelemetry.Ubx.Custom.ClassDefs, as: ClassDefs
  require ViaTelemetry.Ubx.Custom.VehicleCmds.ActuatorCmdDirect, as: ActuatorCmdDirect
  require ViaTelemetry.Ubx.Custom.VehicleCmds.ControllerActuatorOutput, as: ControllerActuatorOutput
  require ViaUtils.Shared.GoalNames, as: SGN
  @cmd_header <<68, 65, 84, 65, 0>>
  @zeros_1 <<0, 0, 0, 0>>
  @zeros_3 <<0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0>>
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
      destination_ip: nil,
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
      request_ip_address_timer: request_ip_address_timer,
      channel_names: Keyword.get(config, :channel_names, %{})
    }

    ViaUtils.Comms.Supervisor.start_operator(__MODULE__)
    ViaUtils.Comms.join_group(__MODULE__, Groups.virtual_uart_actuator_output())

    {:ok, state}
  end

  @impl GenServer
  def handle_info(@request_ip_address_loop, state) do
    state =
      if is_nil(state.destination_ip) do
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
    # Logger.debug("xpi rx: #{data}")
    state = check_for_new_messages_and_process(:binary.bin_to_list(data), state)
    {:noreply, state}
  end

  @impl GenServer
  def handle_cast({:set_ip_address, ip_address}, state) do
    {:noreply, %{state | destination_ip: ip_address}}
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
      case msg_class do
        ClassDefs.vehicle_cmds() ->
          case msg_id do
            ControllerActuatorOutput.id() ->
              cmds =
                UbxInterpreter.deconstruct_message_to_map(
                  ControllerActuatorOutput.bytes(),
                  ControllerActuatorOutput.multipliers(),
                  ControllerActuatorOutput.keys(),
                  payload
                )

              %{
                SGN.aileron_scaled() => aileron_scaled,
                SGN.elevator_scaled() => elevator_scaled,
                SGN.throttle_scaled() => throttle_scaled,
                SGN.rudder_scaled() => rudder_scaled
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

            ActuatorCmdDirect.id() ->
              cmds = ActuatorCmdDirect.get_output_values(payload, state.channel_names)
              # Logger.debug("direct act: #{inspect(cmds)}")
              aileron_scaled = Map.get(cmds, SGN.aileron_scaled())
              elevator_scaled = Map.get(cmds, SGN.elevator_scaled())
              rudder_scaled = Map.get(cmds, SGN.rudder_scaled())

              unless is_nil(aileron_scaled) or is_nil(elevator_scaled) or is_nil(rudder_scaled) do
                send_ail_elev_rud_commands(
                  aileron_scaled,
                  elevator_scaled,
                  rudder_scaled,
                  socket,
                  dest_ip,
                  dest_port
                )
              end

              throttle_scaled = Map.get(cmds, SGN.throttle_scaled())

              unless is_nil(throttle_scaled) do
                throttle_scaled = ViaUtils.Math.get_one_sided_from_two_sided(throttle_scaled)
                send_throttle_command(throttle_scaled, socket, dest_ip, dest_port)
              end

              flaps_scaled = Map.get(cmds, SGN.flaps_scaled())

              unless is_nil(flaps_scaled) do
                flaps_scaled = ViaUtils.Math.get_one_sided_from_two_sided(flaps_scaled)
                send_flaps_command(flaps_scaled, socket, dest_ip, dest_port)
              end

            _other ->
              Logger.warn("Bad message id: #{msg_id}")
          end

        _other ->
          Logger.warn("Bad message class: #{msg_class}")
      end

      check_for_new_messages_and_process([], %{state | ubx: UbxInterpreter.clear(ubx)})
    end
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

    # Logger.debug(
    #   "ail/elev/rud: #{ViaUtils.Format.eftb(aileron_scaled, 3)}/#{ViaUtils.Format.eftb(elevator_scaled, 3)}/#{ViaUtils.Format.eftb(rudder_scaled, 3)}"
    # )
    :gen_udp.send(socket, dest_ip, port, buffer)
  end

  @spec send_throttle_command(number(), any(), tuple(), integer()) :: atom()
  def send_throttle_command(throttle_scaled, socket, dest_ip, port) do
    buffer =
      (@cmd_header <> <<25, 0, 0, 0>>)
      |> Kernel.<>(ViaUtils.Math.uint_from_fp(throttle_scaled, 32))
      |> Kernel.<>(@zeros_7)

    :gen_udp.send(socket, dest_ip, port, buffer)
  end

  @spec send_flaps_command(number(), any(), tuple(), integer()) :: atom()
  def send_flaps_command(flaps_scaled, socket, dest_ip, port) do
    # Logger.error("send flaps: #{flaps_scaled}")
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
end
