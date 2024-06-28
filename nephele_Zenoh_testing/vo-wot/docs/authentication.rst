Authentication Mechanisms
=========================

This page includes information about the authentication mechanisms supported by the VO.

============================================================================ ==== ==== ==== =========
Mechanism                                                                    HTTP CoAP MQTT WebSocket
============================================================================ ==== ==== ==== =========
Basic Authentication                                                         ✔    ✔    ✔
Bearer Token                                                                 ✔
`OpenID for Verifiable Credentials <https://www.w3.org/TR/vc-data-model/>`__ ✔
Oauth2                                                                       ⚠️
============================================================================ ==== ==== ==== =========

⚠️ Oauth2 has not been tested and requires an external endpoint to authenticate tokens.

To set an authentication mechanism and the corresponding credentials, add an entry to the North or South bound
sections of VO/cVO descriptor.

NorthBound Interface
^^^^^^^^^^^^^^^^^^^^

To set the authentication mechanism in the NorthBound interface:

* Basic authentication

.. code-block:: yaml

    BindingNB:
        securityNB:
            security_scheme: basic
            username: <username>
            password: <password>

* Bearer token

.. code-block:: yaml

    BindingNB:
        securityNB:
            security_scheme: bearer
            token: <token_value>

SouthBound Interface
^^^^^^^^^^^^^^^^^^^^

To set the authentication mechanism in the SouthBound interface:

* Basic authentication

.. code-block:: yaml

    BindingSB:
        securitySB:
            securitySBMQTT:
                security_scheme: basic
                username: <username>
                password: <password>
            securitySBHTTP:
                security_scheme: basic
                username: <username>
                password: <password>
            securitySBCOAP:
                security_scheme: basic
                username: <username>
                password: <password>

* Bearer token

.. code-block:: yaml

    BindingSB:
        securitySB:
            securitySBHTTP:
                security_scheme: bearer
                token: <token_value>
            securitySBCOAP:
                security_scheme: bearer
                token: <token_value>

OpenID for Verifiable Credentials
---------------------------------

The current architecture that uses OpenID for Verifiable Credentials can be seen here:

.. image:: images/verifiable_credentials.png

The main components include:

* **Holder**: This component stores and presents all the credentials of the VO/application.

* **Verifier**: This component is responsible for validating the credentials of a request and generating a JWT access token
  that will be attached to each future request.

* **PEP-Proxy**: This component can be used as the entry point of the VO and all traffic needs to go through the proxy
  before reaching the VO. When presented with verifiable credentials, the proxy queries the Verifier with the
  credentials of an incoming request and if valid, a JWT token is returned as a response to the initial request.
  When presented with a JWT token, the token is validated and if valid access is granted
  to the VO.

The source code for the aforementioned components can be found `here <https://gitlab.eclipse.org/eclipse-research-labs/nephele-project/vo-security>`__.

.. note::
    Currently, the credentials are mounted as a directory inside the VO container.
    In the future, the Issuer component can be deployed for example in a Kubernetes cluster
    so that it can issue credentials to the Holder component.

An example is available in the ``examples/plenary-demo-security`` directory. The virtual object
is accompanied by the Verifier, Holder and PEP-Proxy components while an application
that consumes the VO's data, utilizes the Holder component to request the appropriate
tokens to communicate with the VO via the PEP-Proxy component.

.. note::
    In the future all the components will be placed as sidecars of the same Pod. The VO will not
    be exposed directly through a service. All other (c)VO's and applications will first
    have to communicate with the PEP-Proxy.
