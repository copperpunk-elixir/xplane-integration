defmodule XplaneIntegration do
  require Logger
  @moduledoc """
  Documentation for `XplaneIntegration`.
  """

  @doc """
  Hello world.

  ## Examples

      iex> XplaneIntegration.hello()
      :world

  """
  def start_link(config) do
    Logger.debug("Start XplaneIntegration GenServer")
    ViaUtils.Process.start_link_redundant(GenServer, __MODULE__, config, __MODULE__)
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
  def handle_info({:udp, _socket, _src_ip, _src_port, msg}, state) do
    # Logger.debug("received data from #{inspect(src_ip)} on port #{src_port} with length #{length(msg)}")
    # state = parse_data_buffer(msg, state)

    # state =
    #   if state.new_simulation_data_to_publish == true do
    #     # publish_simulation_data(state)
    #     publish_perfect_simulation_data(state)
    #     %{state | new_simulation_data_to_publish: false}
    #   else
    #     state
    #   end

    {:noreply, state}
  end
end
