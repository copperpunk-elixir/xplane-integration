defmodule XplaneIntegration.Send do
  use GenServer
  require Logger
  # alias ViaUtils.Constants, as: VC

  def start_link(config) do
    Logger.debug("Start Simulation.XplaneReceive")
    ViaUtils.Process.start_link_redundant(GenServer, __MODULE__, config, __MODULE__)
  end

  @impl GenServer
  def init(config) do
    port = Keyword.fetch!(config, :port)
    {:ok, socket} = :gen_udp.open(port, broadcast: false, active: true)

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
      new_simulation_data_to_publish: false
    }

    ViaUtils.Comms.Supervisor.start_operator(__MODULE__)

    {:ok, state}
  end
end
