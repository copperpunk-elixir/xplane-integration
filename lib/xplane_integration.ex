defmodule XplaneIntegration do
  use Supervisor
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
    Logger.debug("Start XplaneIntegration Supervisor")
    ViaUtils.Process.start_link_redundant(Supervisor, __MODULE__, config, __MODULE__)
  end

  @impl Supervisor
  def init(config) do
    children = [
      {XplaneIntegration.Receive, config[:receive]},
      {XplaneIntegration.Send, config[:send]},
    ]

    Supervisor.init(children, strategy: :one_for_one)
  end
end
