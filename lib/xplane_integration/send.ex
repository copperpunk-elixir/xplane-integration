defmodule XplaneIntegration.Send do
  use Bitwise
  use GenServer
  require Logger
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
    ViaUtils.Comms.join_group(__MODULE__, :simulation_update_actuators)

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
  def handle_cast({:set_ip_address, ip_address}, state) do
    {:noreply, %{state | destination_ip_address: ip_address}}
  end

  @impl GenServer
  def handle_cast({:simulation_update_actuators, actuators_and_outputs, is_override}, state) do
    # Logger.debug("xp send rx up_act: #{ViaUtils.Format.eftb_map(actuators_and_outputs, 3)}")

    dest_ip = state.destination_ip_address

    unless is_nil(dest_ip) do
      cmds =
        Enum.reduce(actuators_and_outputs, %{}, fn {actuator_name, output}, acc ->
          if is_override do
            case actuator_name do
              :flaps_scaled -> Map.put(acc, actuator_name, get_one_sided_value(output))
              :gear_scaled -> Map.put(acc, actuator_name, get_one_sided_value(output))
              :throttle_scaled -> Map.put(acc, actuator_name, get_one_sided_value(output))
              name -> Map.put(acc, name, output)
            end
          else
            Map.put(acc, actuator_name, output)
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

  @spec send_ail_elev_rud_commands(map(), any(), tuple(), integer()) :: atom()
  def send_ail_elev_rud_commands(commands, socket, dest_ip, port) do
    buffer =
      (@cmd_header <> <<11, 0, 0, 0>>)
      |> Kernel.<>(ViaUtils.Math.uint_from_fp(Map.get(commands, :elevator_scaled, -999), 32))
      |> Kernel.<>(ViaUtils.Math.uint_from_fp(Map.get(commands, :aileron_scaled, -999), 32))
      |> Kernel.<>(ViaUtils.Math.uint_from_fp(Map.get(commands, :rudder_scaled, -999), 32))
      |> Kernel.<>(@zeros_5)

    # Logger.debug("ail/elev/rud: #{Map.get(commands, :aileron)}/#{Map.get(commands, :elevator)}/#{Map.get(commands, :rudder)}")
    # Logger.debug("ail-elev-rud: #{ViaUtils.Format.eftb_map(commands, 3)}")
    :gen_udp.send(socket, dest_ip, port, buffer)
  end

  @spec send_throttle_command(map(), any(), tuple(), integer()) :: atom()
  def send_throttle_command(commands, socket, dest_ip, port) do
    buffer =
      (@cmd_header <> <<25, 0, 0, 0>>)
      |> Kernel.<>(ViaUtils.Math.uint_from_fp(Map.get(commands, :throttle_scaled, -999), 32))
      |> Kernel.<>(@zeros_7)

    # |> Kernel.<>(<<0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0>>)
    # Logger.debug("buffer: #{buffer}")
    # Logger.debug("thr: #{Map.get(commands, :throttle_scaled)}")
    :gen_udp.send(socket, dest_ip, port, buffer)
  end

  @spec send_flaps_command(map(), any(), tuple(), integer()) :: atom()
  def send_flaps_command(commands, socket, dest_ip, port) do
    # Logger.error("flaps: #{Map.get(commands, :flaps_scaled)}")
    buffer =
      (@cmd_header <> <<13, 0, 0, 0>>)
      |> Kernel.<>(@zeros_3)
      |> Kernel.<>(ViaUtils.Math.uint_from_fp(Map.get(commands, :flaps_scaled, -999), 32))
      |> Kernel.<>(ViaUtils.Math.uint_from_fp(-999, 32))
      |> Kernel.<>(ViaUtils.Math.uint_from_fp(-999, 32))
      |> Kernel.<>(@zeros_1)
      |> Kernel.<>(ViaUtils.Math.uint_from_fp(-999, 32))

    :gen_udp.send(socket, dest_ip, port, buffer)
  end

  @spec send_ail_elev_rud_commands_test(map()) :: atom()
  def send_ail_elev_rud_commands_test(commands) do
    GenServer.cast(__MODULE__, {:send_commands, :ail_elev_rud, commands})
  end

  @spec send_throttle_command_test(map()) :: atom()
  def send_throttle_command_test(commands) do
    GenServer.cast(__MODULE__, {:send_commands, :throttle, commands})
  end

  @spec send_flaps_command_test(map()) :: atom()
  def send_flaps_command_test(commands) do
    GenServer.cast(__MODULE__, {:send_commands, :flaps, commands})
  end

  @spec get_one_sided_value(number()) :: number()
  def get_one_sided_value(two_sided_value) do
    0.5 * two_sided_value + 0.5
  end
end
