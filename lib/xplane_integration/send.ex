defmodule XplaneIntegration.Send do
  require Logger
  alias ViaUtils.Constants, as: VC

  def start_link(config) do
    Logger.debug("Start Simulation.XplaneReceive")
    Common.Utils.start_link_redundant(GenServer, __MODULE__, nil, __MODULE__)
  end

  @impl GenServer
  def init(_) do
    {:ok, %{}}
  end

  @impl GenServer
  def terminate(reason, state) do
    Logging.Logger.log_terminate(reason, state, __MODULE__)
    state
  end

  @impl GenServer
  def handle_cast({:begin, config}, _state) do
    port = Keyword.fetch!(config, :port)
    {:ok, socket} = :gen_udp.open(port, [broadcast: false, active: true])
    state = %{
      socket: socket,
      port: port,
      bodyaccel: %{},
      attitude: %{},
      bodyrate: %{},
      position: %{},
      velocity: %{},
      agl: 0,
      airspeed: 0,
      new_simulation_data_to_publish: false,
    }
    ViaUtils.Comms.start_link
    Comms.System.start_operator(__MODULE__)
    {:noreply, state}
  end

  end
