from pushpak_brain.failsafe import isolation_state


def test_never_heard_is_isolated():
    assert isolation_state(None, [], 2.0) == (True, 'no /pushpak/peers_alive update')


def test_stale_liveness_topic_is_isolated_even_with_old_peers():
    assert isolation_state(2.5, [0], 2.0)[0] is True
    assert isolation_state(float('nan'), [0], 2.0)[0] is True


def test_empty_peer_list_is_isolated():
    assert isolation_state(0.1, [], 2.0) == (True, 'no peer alive')


def test_any_peer_alive_clears():
    assert isolation_state(0.1, [0], 2.0) == (False, '')
    assert isolation_state(1.9, [3, 4], 2.0) == (False, '')
    assert isolation_state(-0.01, [0], 2.0) == (False, '')
