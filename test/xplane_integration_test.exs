defmodule XplaneIntegrationTest do
  use ExUnit.Case
  doctest XplaneIntegration

  test "greets the world" do
    assert XplaneIntegration.hello() == :world
  end
end
